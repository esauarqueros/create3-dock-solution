import math

from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray

from .controller import DockController
from .detector import DockDetector
from .types import Pose2D

# Valores por defecto de los parámetros del detector -- deben reflejar
# config/dock_detector.yaml (grupo `detector`). Ver create3_dock_solution/perception/dock_model.py.
_DETECTOR_PARAM_DEFAULTS = {
    'dock_geometry': {
        'box_size': 0.08, 'box_gap': 0.095, 'box_protrusion': 0.08,
        'wall_search_range_max': 3.0, 'wall_half_extent': 0.35, 'template_point_spacing': 0.01,
    },
    'ransac': {
        'max_iterations': 200, 'inlier_threshold': 0.015,
        'min_inlier_ratio': 0.35, 'min_inliers': 15,
    },
    'box_validation': {
        'gap_tolerance': 0.02, 'protrusion_tolerance': 0.02, 'width_tolerance': 0.02,
        'cluster_max_gap': 0.03, 'cluster_min_points': 2,
        'min_protrusion_m': 0.03, 'max_protrusion_m': 0.15,
    },
    'icp': {
        'enable': True, 'coarse_to_fine_range_m': 1.0, 'max_iterations': 25,
        'max_correspondence_dist': 0.05, 'convergence_translation_eps': 0.002,
        'convergence_rotation_eps_rad': 0.01, 'min_correspondences': 10, 'max_residual_rms': 0.02,
        'max_translation_correction_m': 0.15, 'max_rotation_correction_rad': 0.52,
    },
    'filter': {
        'alpha_min': 0.05, 'alpha_max': 0.6, 'confidence_decay_per_s': 0.5,
        'lost_timeout_s': 1.5, 'max_jump_m': 0.5, 'outlier_reject_confidence': 0.3,
    },
}


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _declare_detector_params(node: Node) -> dict:
    params: dict = {}
    for group, defaults in _DETECTOR_PARAM_DEFAULTS.items():
        params[group] = {}
        for key, default in defaults.items():
            full_name = f'detector.{group}.{key}'
            node.declare_parameter(full_name, default)
            params[group][key] = node.get_parameter(full_name).value
    return params


class DockNode(Node):
    def __init__(self):
        super().__init__('dock_lidar')

        detector_params = _declare_detector_params(self)
        self.detector = DockDetector(params=detector_params)
        self.controller = DockController(params={})

        self.declare_parameter('frames.base_frame', 'base_link')
        self.declare_parameter('frames.odom_frame', 'odom')
        self.declare_parameter('debug.enable_markers', True)
        self.declare_parameter('debug.marker_topic', 'dock_detector/markers')

        self.base_frame = self.get_parameter('frames.base_frame').value
        self.odom_frame = self.get_parameter('frames.odom_frame').value
        self._enable_markers = self.get_parameter('debug.enable_markers').value

        self.robot_pose: Pose2D | None = None
        self.dock = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._laser_to_base: Pose2D | None = None

        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        # + /dock_status para is_docked (a cargo del compañero del controller, en on_timer)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        marker_topic = self.get_parameter('debug.marker_topic').value
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)

        self.create_timer(0.05, self.on_timer)   # 20 Hz, publicar siempre

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
        self.dock = self.detector.update(
            ranges, msg.angle_min, msg.angle_increment, self.robot_pose, stamp,
            laser_to_base=laser_to_base, range_min=msg.range_min, range_max=msg.range_max,
        )

        if self._enable_markers:
            self._publish_debug_markers()

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.robot_pose = Pose2D(x=p.x, y=p.y, yaw=_yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def on_timer(self):
        ...   # controller.step(...) -> publica Twist (a cargo del compañero del controller)

    def _publish_debug_markers(self):
        markers = MarkerArray()
        debug = self.detector.last_debug
        stamp = self.get_clock().now().to_msg()

        wall = debug.get('wall')
        if wall is not None:
            markers.markers.append(self._wall_marker(wall, stamp))

        coarse_pose = debug.get('coarse_pose')
        if coarse_pose is not None:
            markers.markers.append(self._axis_marker(
                coarse_pose, self.base_frame, stamp, 2, (1.0, 0.6, 0.0)))

        if self.dock is not None:
            markers.markers.append(self._axis_marker(
                self.dock.pose, self.odom_frame, stamp, 1, (0.0, 1.0, 0.0)))

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
        m.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=1.0)
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


def main():
    rclpy.init()
    rclpy.spin(DockNode())
    rclpy.shutdown()
