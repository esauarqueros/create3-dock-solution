import math
import signal

from geometry_msgs.msg import Point, Twist
from irobot_create_msgs.msg import DockStatus, HazardDetection, HazardDetectionVector
from nav_msgs.msg import Odometry
import numpy as np
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray

from .control.dock_targets import dock_goal_from_marker, predock_from_goal
from .control.params import ControllerParams
from .controller import DockController, DOCKED, FAILED
from .detector import DockDetector
from .diagnostics.run_recorder import RunRecorder
from .perception.odom_history import OdomHistory
from .types import HazardState, Pose2D

# Valores por defecto de los parámetros del detector -- deben reflejar
# config/dock_detector.yaml (grupo `detector`). Ver create3_dock_solution/perception/dock_model.py.
_DETECTOR_PARAM_DEFAULTS = {
    'dock_geometry': {
        'box_size': 0.08, 'box_gap': 0.095, 'box_protrusion': 0.08,
        'wall_search_range_max': 3.0, 'wall_half_extent': 0.35, 'template_point_spacing': 0.01,
    },
    'ransac': {
        'max_iterations': 200, 'inlier_threshold': 0.015,
        'min_inlier_ratio': 0.10, 'min_inliers': 15, 'max_wall_candidates': 3,
        'confidence_inlier_ratio_ref': 0.35,
    },
    'box_validation': {
        'gap_tolerance': 0.02, 'protrusion_tolerance': 0.02, 'width_tolerance': 0.02,
        'cluster_max_gap': 0.03, 'cluster_min_points': 2,
        'min_protrusion_m': 0.03, 'max_protrusion_m': 0.15,
        'gap_tolerance_range_factor': 2.5, 'width_tolerance_range_factor': 2.0,
        'protrusion_tolerance_range_factor': 1.5, 'incidence_factor': 2.0,
        'cluster_min_points_far': 1, 'cluster_min_points_far_range_m': 1.8,
    },
    'icp': {
        'enable': True, 'coarse_to_fine_range_m': 1.0, 'estimate_yaw': False,
        'max_iterations': 25,
        'max_correspondence_dist': 0.05, 'convergence_translation_eps': 0.002,
        'convergence_rotation_eps_rad': 0.01, 'min_correspondences': 10, 'max_residual_rms': 0.02,
        'max_translation_correction_m': 0.15, 'max_rotation_correction_rad': 0.52,
    },
    'filter': {
        'alpha_min': 0.05, 'alpha_max': 0.6, 'confidence_decay_per_s': 0.5,
        'lost_timeout_s': 1.5, 'max_jump_m': 0.5, 'outlier_reject_confidence': 0.3,
        'reinit_after_consecutive': 3, 'reinit_consistency_m': 0.10,
    },
    'deskew': {'enable': True},
}


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _declare_group_params(node: Node, prefix: str, defaults: dict) -> dict:
    """Declara `<prefix>.<grupo>.<clave>` y devuelve los valores como dict anidado."""
    params: dict = {}
    for group, group_defaults in defaults.items():
        params[group] = {}
        for key, default in group_defaults.items():
            full_name = f'{prefix}.{group}.{key}'
            node.declare_parameter(full_name, default)
            params[group][key] = node.get_parameter(full_name).value
    return params


# Colores de los markers (RGB). Ver docs/LANZAMIENTO.md.
_COLOR_WALL = (0.2, 0.6, 1.0)       # azul claro: pared elegida (base_link)
_COLOR_COARSE = (1.0, 0.6, 0.0)     # naranja: pose gruesa del detector (base_link)
_COLOR_MARKER = (0.0, 1.0, 0.0)     # verde: pose filtrada del marcador (odom)
_COLOR_GOAL = (1.0, 0.0, 0.0)       # rojo: dock real, mira a la pared (odom)
_COLOR_PREDOCK = (1.0, 0.0, 1.0)    # magenta: pre-dock (odom)

_HAZARD_FIELDS = {
    HazardDetection.BUMP: 'bump', HazardDetection.STALL: 'stall',
    HazardDetection.CLIFF: 'cliff', HazardDetection.WHEEL_DROP: 'cliff',
    HazardDetection.OBJECT_PROXIMITY: 'proximity',
}


class DockNode(Node):
    def __init__(self):
        super().__init__('dock_lidar')

        detector_params = _declare_group_params(self, 'detector', _DETECTOR_PARAM_DEFAULTS)
        self.detector = DockDetector(params=detector_params)

        controller_params = ControllerParams.from_dict(_declare_group_params(
            self, 'controller', ControllerParams().to_nested_dict()))
        for error in controller_params.validate():
            self.get_logger().error(f'Parámetro inválido del controller: {error}')
        self.controller = DockController(controller_params, log=self.get_logger().info)

        self.declare_parameter('frames.base_frame', 'base_link')
        self.declare_parameter('frames.odom_frame', 'odom')
        self.declare_parameter('debug.enable_markers', True)
        self.declare_parameter('debug.marker_topic', 'dock_detector/markers')
        self.declare_parameter('debug.run_report.enable', False)
        self.declare_parameter('debug.run_report.output_dir', '/ws/dock_runs')
        self.declare_parameter('debug.run_report.save_plot', True)
        self.declare_parameter('debug.run_report.tag', '')
        self.declare_parameter('odom.history_s', 1.0)
        self.declare_parameter('odom.max_extrapolation_s', 0.1)

        self.base_frame = self.get_parameter('frames.base_frame').value
        self.odom_frame = self.get_parameter('frames.odom_frame').value
        self._enable_markers = self.get_parameter('debug.enable_markers').value

        self.recorder: RunRecorder | None = None
        if self.get_parameter('debug.run_report.enable').value:
            self.recorder = RunRecorder(
                self.get_parameter('debug.run_report.output_dir').value,
                tag=self.get_parameter('debug.run_report.tag').value,
                save_plot=self.get_parameter('debug.run_report.save_plot').value,
                log=self.get_logger().info)

        self.robot_pose: Pose2D | None = None
        # Pose del robot en el instante de cada scan (no la última odometría).
        self.odom_history = OdomHistory(
            max_age_s=self.get_parameter('odom.history_s').value,
            max_extrapolation_s=self.get_parameter('odom.max_extrapolation_s').value)
        self.dock = None
        # Reloj del nodo al recibir el último scan con detección: la antigüedad
        # del dock no depende de que el stamp del scan use el mismo reloj.
        self._dock_received_at: float | None = None
        self.is_docked = False
        self._hazard_seen: dict[str, float] = {}

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._laser_to_base: Pose2D | None = None

        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        # Solo is_docked: dock_visible sale de los sensores IR (prohibidos).
        self.create_subscription(
            DockStatus, '/dock_status', self.on_dock_status, qos_profile_sensor_data)
        self.create_subscription(
            HazardDetectionVector, '/hazard_detection', self.on_hazard, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        marker_topic = self.get_parameter('debug.marker_topic').value
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)

        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.create_timer(0.05, self.on_timer)   # 20 Hz, publicar siempre
        # Reloj de pared: con use_sim_time y sin /clock los timers del nodo no corren.
        self._clock_check_timer = self.create_timer(
            3.0, self._check_clock_source, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _check_clock_source(self):
        self._clock_check_timer.cancel()
        use_sim_time = self.get_parameter('use_sim_time').value
        clock_publishers = self.count_publishers('/clock')
        if use_sim_time and clock_publishers == 0:
            self.get_logger().warn(
                'use_sim_time=true pero nadie publica /clock: el nodo no avanzará. '
                '¿Robot real? Lanza con use_sim_time:=false')
        elif not use_sim_time and clock_publishers > 0:
            self.get_logger().warn(
                'use_sim_time=false pero hay /clock (simulación): lanza con use_sim_time:=true')

    def _on_set_parameters(self, params) -> SetParametersResult:
        """Ajuste en caliente de `controller.*` y `debug.enable_markers`."""
        changes = {p.name: p.value for p in params}
        controller_changes = {k: v for k, v in changes.items() if k.startswith('controller.')}
        if controller_changes:
            nested = self.controller.p.to_nested_dict()
            for name, value in controller_changes.items():
                _, group, key = name.split('.', 2)
                if group in nested and key in nested[group]:
                    nested[group][key] = value
            new_params = ControllerParams.from_dict(nested)
            errors = new_params.validate()
            if errors:
                return SetParametersResult(successful=False, reason='; '.join(errors))
            self.controller.set_params(new_params)
            self.get_logger().info(f'Parámetros del controller actualizados: {controller_changes}')
        if 'debug.enable_markers' in changes:
            self._enable_markers = bool(changes['debug.enable_markers'])
        return SetParametersResult(successful=True)

    def _resolve_laser_to_base(self, laser_frame: str) -> Pose2D | None:
        """
        Resuelve y cachea la transformada estática laser_link -> base_link.

        El LIDAR está desplazado y rotado 180° respecto al robot; se resuelve vía
        TF (no hardcodeado) y se cachea: no hace falta volver a consultarla en cada
        scan, y así seguimos funcionando aunque el montaje físico cambie sin tocar código.
        """
        if self._laser_to_base is not None:
            return self._laser_to_base
        tf_errors = (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                     tf2_ros.ExtrapolationException)
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, laser_frame, Time())
        except tf_errors:
            self.get_logger().warn(
                f'TF {laser_frame} -> {self.base_frame} aún no disponible',
                throttle_duration_sec=2.0)
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        self._laser_to_base = Pose2D(x=t.x, y=t.y, yaw=_yaw_from_quaternion(q.x, q.y, q.z, q.w))
        return self._laser_to_base

    def on_scan(self, msg: LaserScan):
        if self.robot_pose is None:
            return
        laser_to_base = self._resolve_laser_to_base(msg.header.frame_id)
        if laser_to_base is None:
            return

        ranges = np.asarray(msg.ranges, dtype=float)
        stamp = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        robot_pose = self.odom_history.pose_at(stamp)
        if robot_pose is None:
            # Stamps de scan y odom fuera del historial (p.ej. relojes de LIDAR y
            # robot sin sincronizar en el robot real): mejor la última odometría
            # que descartar el scan.
            self.get_logger().warn(
                'Sin odometría en el instante del scan; se usa la última recibida',
                throttle_duration_sec=5.0)
            robot_pose = self.robot_pose
        dock = self.detector.update(
            ranges, msg.angle_min, msg.angle_increment, robot_pose, stamp,
            laser_to_base=laser_to_base, range_min=msg.range_min, range_max=msg.range_max,
            time_increment=msg.time_increment, twist=self.odom_history.twist_at(stamp),
        )
        self.dock = dock
        if dock is not None:
            self._dock_received_at = self._now()

        if self._enable_markers:
            self._publish_debug_markers(msg.header.stamp)

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.robot_pose = Pose2D(x=p.x, y=p.y, yaw=_yaw_from_quaternion(q.x, q.y, q.z, q.w))
        stamp = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        self.odom_history.add(
            stamp, self.robot_pose, msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def on_dock_status(self, msg: DockStatus):
        self.is_docked = bool(msg.is_docked)

    def on_hazard(self, msg: HazardDetectionVector):
        now = self._now()
        for detection in msg.detections:
            field = _HAZARD_FIELDS.get(detection.type)
            if field is not None:
                self._hazard_seen[field] = now

    def _hazards(self, now: float) -> HazardState:
        hold = self.controller.p.hazard.hold_s
        active = {k for k, t in self._hazard_seen.items() if now - t <= hold}
        return HazardState(bump='bump' in active, stall='stall' in active,
                           cliff='cliff' in active, proximity='proximity' in active)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_timer(self):
        if not self.controller.p.supervision.enable:
            return      # modo solo detector: no publicar para no pelear con teleop
        now = self._now()
        twist = Twist()     # por defecto: quieto
        if self.robot_pose is not None and now > 0.0:
            if self.recorder is not None and not self.recorder.started:
                self.recorder.start(now, self.controller.p.approach.mode,
                                    self.controller.p.to_nested_dict())
            dock = self.dock
            max_age = self.controller.p.supervision.dock_max_age_s
            if dock is not None and (self._dock_received_at is None
                                     or now - self._dock_received_at > max_age):
                dock = None     # /scan dejó de llegar
            hazards = self._hazards(now)
            cmd = self.controller.step(dock, self.robot_pose, self.is_docked, hazards, now)
            twist.linear.x = float(cmd.v)
            twist.angular.z = float(cmd.w)
            self._log_status()
            if self.recorder is not None:
                self.recorder.record(now, self.robot_pose, self.controller.last_debug, hazards)
                if self.controller.state in (DOCKED, FAILED):
                    self.finish_report(self.controller.state.lower())
        self.cmd_pub.publish(twist)

    def _log_status(self):
        d = self.controller.last_debug
        if d.get('e_long') is None:
            text = f"{d['state']} | sin objetivo"
        else:
            text = (f"{d['state']} [{d['mode']}] | e_long={d['e_long']:+.3f} m "
                    f"e_lat={d['e_lat'] * 1000:+.1f} mm e_yaw={math.degrees(d['e_yaw']):+.1f}° "
                    f"| v={d['v']:.2f} w={d['w']:+.2f} | intentos={d['attempts']}")
        self.get_logger().info(text, throttle_duration_sec=1.0)

    def finish_report(self, status: str):
        if self.recorder is not None and self.recorder.started and not self.recorder.finished:
            self.recorder.finish(status, self.controller.events, self.controller.t_start,
                                 self.controller.attempts)

    def _publish_debug_markers(self, stamp):
        # Sellados con el stamp del scan: coincide con el reloj de TF/RViz tanto
        # en simulación (use_sim_time) como en el robot real.
        markers = MarkerArray()
        debug = self.detector.last_debug

        wall = debug.get('wall')
        if wall is not None:
            markers.markers.append(self._wall_marker(wall, stamp))

        coarse_pose = debug.get('coarse_pose')
        if coarse_pose is not None:
            markers.markers.append(self._axis_marker(
                coarse_pose, self.base_frame, stamp, 2, _COLOR_COARSE))

        if self.dock is not None:
            markers.markers.append(self._axis_marker(
                self.dock.pose, self.odom_frame, stamp, 1, _COLOR_MARKER))

        goal = self.controller.last_debug.get('goal')
        predock = self.controller.last_debug.get('predock')
        if not self.controller.p.supervision.enable and self.dock is not None:
            # Modo solo detector: el controller no corre, pero igual se muestran
            # el dock real y el pre-dock para validar la geometría con teleop.
            g = self.controller.p.geometry
            goal = dock_goal_from_marker(self.dock.pose, g.dock_offset_m)
            predock = predock_from_goal(goal, g.predock_distance_m)
        if goal is not None:
            markers.markers.append(self._axis_marker(
                goal, self.odom_frame, stamp, 3, _COLOR_GOAL))
        if predock is not None:
            markers.markers.append(self._axis_marker(
                predock, self.odom_frame, stamp, 4, _COLOR_PREDOCK))

        self.marker_pub.publish(markers)

    def _wall_marker(self, wall, stamp) -> Marker:
        m = Marker()
        m.header.frame_id = self.base_frame
        m.header.stamp = stamp
        m.ns = 'dock_detector'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.01
        m.color = ColorRGBA(r=_COLOR_WALL[0], g=_COLOR_WALL[1], b=_COLOR_WALL[2], a=1.0)
        tangent = np.array([-wall.normal[1], wall.normal[0]])
        for s in (-1.0, 1.0):
            p = wall.point + s * 0.5 * tangent
            m.points.append(Point(x=float(p[0]), y=float(p[1]), z=0.0))
        return m

    @staticmethod
    def _axis_marker(pose: Pose2D, frame_id: str, stamp, marker_id: int, color) -> Marker:
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = stamp
        m.ns = 'dock_detector'
        m.id = marker_id
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.scale.x, m.scale.y, m.scale.z = 0.15, 0.03, 0.03
        m.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.0)
        m.pose.position.x = pose.x
        m.pose.position.y = pose.y
        m.pose.orientation.z = math.sin(pose.yaw / 2.0)
        m.pose.orientation.w = math.cos(pose.yaw / 2.0)
        return m


def main(args=None):
    # Sin el manejador de señales de rclpy: tras Ctrl+C el contexto sigue vivo
    # para publicar un Twist cero y cerrar el reporte.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = DockNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # launch puede reenviar SIGINT: no interrumpir el apagado a la mitad.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            node.cmd_pub.publish(Twist())
            node.finish_report('aborted')
        except Exception as exc:   # noqa: B902 -- apagado best-effort
            node.get_logger().warn(f'Error al apagar: {exc}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
