import math

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry


def wrap(angle):
    """Normaliza un ángulo a [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class ControlNode(Node):
    """
    Docking:

        GO_TO_PREDOCK -> ALIGN_AT_PREDOCK -> FINAL_APPROACH -> DOCKED
                                                    \\-> (tras N intentos) FAILED

    Fuente de la pose del dock:
      - use_perception:=false (por defecto): pose fija (dock_x, dock_y, dock_theta_deg).
      - use_perception:=true : se suscribe a /dock_pose (PoseStamped, frame odom).

    Convención: dock_theta es la dirección en que el ROBOT viaja para entrar al dock.
    """

    STATES_WITH_TIMEOUT = ("GO_TO_PREDOCK", "ALIGN_AT_PREDOCK", "FINAL_APPROACH")

    def __init__(self):
        super().__init__('control_node')

        # ---------------- Parámetros ROS ----------------
        self.declare_parameter('use_perception', False)
        self.declare_parameter('dock_topic', '/dock_pose')
        self.declare_parameter('dock_frame', 'odom')
        self.declare_parameter('dock_x', 1.0)
        self.declare_parameter('dock_y', 0.2)
        self.declare_parameter('dock_theta_deg', 90.0)

        self.use_perception = self.get_parameter('use_perception').value
        dock_topic = self.get_parameter('dock_topic').value
        self.dock_frame = self.get_parameter('dock_frame').value

        # ---------------- Comunicación ----------------
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        if self.use_perception:
            self.dock_sub = self.create_subscription(
                PoseStamped, dock_topic, self.dock_pose_callback, 10
            )
        self.timer = self.create_timer(0.1, self.control_loop)

        # ---------------- Pose del robot ----------------
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.odom_received = False

        # ---------------- Pose del dock / pre-dock ----------------
        self.d_pre = 0.50  # m detrás del dock, sobre el eje
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_theta = 0.0
        self.pre_x = 0.0
        self.pre_y = 0.0
        self.dock_received = False
        self.target_frozen = False

        # Filtro / rechazo de estimaciones (solo con percepción)
        self.alpha = 0.3                        # suavizado exponencial
        self.max_jump = 0.30                    # m
        self.max_jump_yaw = math.radians(30.0)
        self.max_rejected = 5                   # rechazos seguidos antes de aceptar el cambio
        self.rejected_count = 0

        if not self.use_perception:
            self.set_dock_target(
                self.get_parameter('dock_x').value,
                self.get_parameter('dock_y').value,
                math.radians(self.get_parameter('dock_theta_deg').value),
            )
            self.dock_received = True

        # ---------------- GO_TO_PREDOCK ----------------
        self.k_v = 0.4
        self.k_alpha = 1.5
        self.v_max = 0.25
        self.v_min = 0.03
        self.w_max_go = 1.0
        self.predock_tol = 0.03
        self.rotate_in_place_angle = math.radians(60.0)

        # ---------------- ALIGN_AT_PREDOCK ----------------
        self.k_theta = 1.0
        self.w_max_align = 0.6
        self.w_min_align = 0.08
        self.align_tol = math.radians(3.0)

        # ---------------- FINAL_APPROACH ----------------
        self.v_dock = 0.03
        self.v_dock_slow = 0.015
        self.slow_zone = 0.10
        self.dock_distance = 0.03
        self.k_lat = 5.0
        self.max_lat_offset = math.radians(20.0)
        self.k_yaw = 1.5
        self.w_max_dock = 0.5
        self.abort_lat = 0.08
        self.abort_yaw = math.radians(25.0)

        # ---------------- Robustez ----------------
        self.max_attempts = 3
        self.attempts = 0
        self.state_timeouts = {
            "GO_TO_PREDOCK": 90.0,
            "ALIGN_AT_PREDOCK": 20.0,
            "FINAL_APPROACH": 60.0,
        }
        self.progress_eps = 0.02        # m de mejora mínima en e_long
        self.progress_timeout = 10.0    # s sin mejorar -> abortar intento
        self.best_e_long = -float('inf')
        self.best_e_long_time = 0.0
        self.fail_reason = ""

        # ---------------- Estado ----------------
        self.state = "GO_TO_PREDOCK"
        self.state_start = self.now_s()
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.print_counter = 0

        src = 'PERCEPTION ' + dock_topic if self.use_perception else 'FIXED POSE'
        self.get_logger().info(f'Control node started | dock source: {src}')

    # ------------------------------------------------------------------
    # Utilidades
    # ------------------------------------------------------------------
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def set_dock_target(self, x, y, theta):
        """Actualiza la pose del dock y recalcula el pre-dock sobre su eje."""
        self.target_x = x
        self.target_y = y
        self.target_theta = theta
        self.pre_x = x - self.d_pre * math.cos(theta)
        self.pre_y = y - self.d_pre * math.sin(theta)

    def change_state(self, new_state):
        self.get_logger().info(f"{self.state} -> {new_state}")
        self.state = new_state
        self.state_start = self.now_s()

        if new_state == "GO_TO_PREDOCK":
            self.target_frozen = False          # se puede refinar la estimación
        elif new_state == "ALIGN_AT_PREDOCK":
            self.target_frozen = True           # objetivo fijo desde aquí
        elif new_state == "FINAL_APPROACH":
            self.best_e_long = -float('inf')
            self.best_e_long_time = self.now_s()

    def abort_attempt(self, reason):
        """Cuenta un intento fallido; reintenta o pasa a FAILED."""
        self.attempts += 1
        if self.attempts >= self.max_attempts:
            self.fail(f"{reason} (attempt {self.attempts}/{self.max_attempts})")
        else:
            self.get_logger().warn(
                f"{reason} -> retry {self.attempts}/{self.max_attempts}, "
                f"back to GO_TO_PREDOCK"
            )
            self.change_state("GO_TO_PREDOCK")

    def fail(self, reason):
        self.fail_reason = reason
        self.get_logger().error(f"DOCKING FAILED: {reason}")
        self.change_state("FAILED")

    # ------------------------------------------------------------------
    # Errores en el marco del dock
    # ------------------------------------------------------------------
    def dock_frame_errors(self):
        """
        e_long: negativo = aún no llega al dock, 0 = en el dock
        e_lat : desviación lateral respecto al eje (positivo = izquierda)
        e_yaw : target_theta - yaw
        """
        dx = self.robot_x - self.target_x
        dy = self.robot_y - self.target_y
        c = math.cos(self.target_theta)
        s = math.sin(self.target_theta)

        e_long = c * dx + s * dy
        e_lat = -s * dx + c * dy
        e_yaw = wrap(self.target_theta - self.robot_yaw)
        return e_long, e_lat, e_yaw

    # ------------------------------------------------------------------
    # Callback de percepción (solo si use_perception:=true)
    # ------------------------------------------------------------------
    def dock_pose_callback(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.dock_frame:
            self.get_logger().warn(
                f"Dock pose in frame '{msg.header.frame_id}', expected "
                f"'{self.dock_frame}'. Ignored.",
                throttle_duration_sec=5.0
            )
            return

        # Objetivo congelado: se ignoran estimaciones nuevas
        if self.target_frozen:
            return

        x = msg.pose.position.x
        y = msg.pose.position.y
        th = quat_to_yaw(msg.pose.orientation)

        # Primera estimación: se acepta tal cual
        if not self.dock_received:
            self.set_dock_target(x, y, th)
            self.dock_received = True
            self.get_logger().info(
                f"First dock estimate: ({x:.2f}, {y:.2f}, {math.degrees(th):.1f} deg)"
            )
            return

        # Rechazo de saltos bruscos (outliers)
        jump = math.hypot(x - self.target_x, y - self.target_y)
        jump_yaw = abs(wrap(th - self.target_theta))
        if jump > self.max_jump or jump_yaw > self.max_jump_yaw:
            self.rejected_count += 1
            if self.rejected_count < self.max_rejected:
                self.get_logger().warn(
                    f"Dock estimate jump rejected ({jump:.2f} m, "
                    f"{math.degrees(jump_yaw):.0f} deg)",
                    throttle_duration_sec=2.0
                )
                return
            # El cambio es persistente: se acepta sin filtrar
            self.rejected_count = 0
            self.set_dock_target(x, y, th)
            self.get_logger().warn("Persistent dock change accepted")
            return

        self.rejected_count = 0

        # Suavizado exponencial
        a = self.alpha
        nx = self.target_x + a * (x - self.target_x)
        ny = self.target_y + a * (y - self.target_y)
        nth = wrap(self.target_theta + a * wrap(th - self.target_theta))
        self.set_dock_target(nx, ny, nth)

    # ------------------------------------------------------------------
    # Lazo de control
    # ------------------------------------------------------------------
    def control_loop(self):
        if not self.odom_received:
            return

        if not self.dock_received:
            self.get_logger().info(
                "Waiting for dock pose...", throttle_duration_sec=5.0
            )
            return

        now = self.now_s()
        e_long, e_lat, e_yaw = self.dock_frame_errors()

        pdx = self.pre_x - self.robot_x
        pdy = self.pre_y - self.robot_y
        predock_dist = math.hypot(pdx, pdy)
        predock_heading_err = wrap(math.atan2(pdy, pdx) - self.robot_yaw)

        self.v_cmd = 0.0
        self.w_cmd = 0.0

        # ---------------- Timeout por estado ----------------
        if self.state in self.STATES_WITH_TIMEOUT:
            limit = self.state_timeouts[self.state]
            if now - self.state_start > limit:
                self.fail(f"timeout in {self.state} (> {limit:.0f} s)")

        # ---------------- GO_TO_PREDOCK ----------------
        if self.state == "GO_TO_PREDOCK":

            if predock_dist < self.predock_tol:
                self.change_state("ALIGN_AT_PREDOCK")
            else:
                self.w_cmd = clamp(
                    self.k_alpha * predock_heading_err,
                    -self.w_max_go, self.w_max_go
                )
                if abs(predock_heading_err) > self.rotate_in_place_angle:
                    self.v_cmd = 0.0
                else:
                    v = self.k_v * predock_dist * max(0.0, math.cos(predock_heading_err))
                    self.v_cmd = clamp(v, self.v_min, self.v_max)
                    self.v_cmd = min(self.v_cmd, max(predock_dist * 2.0, 0.01))

        # ---------------- ALIGN_AT_PREDOCK ----------------
        elif self.state == "ALIGN_AT_PREDOCK":

            if abs(e_lat) > self.abort_lat:
                self.abort_attempt(f"off axis at pre-dock (e_lat={e_lat:.3f} m)")
            elif abs(e_yaw) < self.align_tol:
                self.change_state("FINAL_APPROACH")
            else:
                w = self.k_theta * e_yaw
                if abs(w) < self.w_min_align:
                    w = math.copysign(self.w_min_align, e_yaw)
                self.w_cmd = clamp(w, -self.w_max_align, self.w_max_align)

        # ---------------- FINAL_APPROACH ----------------
        elif self.state == "FINAL_APPROACH":

            if e_long >= -self.dock_distance:
                self.get_logger().info(
                    f"DOCKED | e_long={e_long:.3f} m, e_lat={e_lat:.3f} m, "
                    f"e_yaw={math.degrees(e_yaw):.2f} deg"
                )
                self.change_state("DOCKED")

            elif abs(e_lat) > self.abort_lat or abs(e_yaw) > self.abort_yaw:
                self.abort_attempt(
                    f"abort final approach (e_lat={e_lat:.3f} m, "
                    f"e_yaw={math.degrees(e_yaw):.1f} deg)"
                )

            else:
                # Watchdog de progreso: e_long debe ir mejorando
                if e_long > self.best_e_long + self.progress_eps:
                    self.best_e_long = e_long
                    self.best_e_long_time = now
                elif now - self.best_e_long_time > self.progress_timeout:
                    self.abort_attempt(
                        f"no progress in final approach for "
                        f"{self.progress_timeout:.0f} s"
                    )

                # Solo se calcula el comando si seguimos en FINAL_APPROACH
                if self.state == "FINAL_APPROACH":
                    lat_offset = clamp(
                        math.atan(self.k_lat * e_lat),
                        -self.max_lat_offset, self.max_lat_offset
                    )
                    yaw_ref = self.target_theta - lat_offset
                    yaw_err_eff = wrap(yaw_ref - self.robot_yaw)

                    self.w_cmd = clamp(
                        self.k_yaw * yaw_err_eff, -self.w_max_dock, self.w_max_dock
                    )
                    v = self.v_dock_slow if e_long > -self.slow_zone else self.v_dock
                    self.v_cmd = v * max(0.0, math.cos(yaw_err_eff))

        # ---------------- DOCKED / FAILED ----------------
        elif self.state in ("DOCKED", "FAILED"):
            self.v_cmd = 0.0
            self.w_cmd = 0.0

        # ---------------- Log (1 Hz) ----------------
        self.print_counter += 1
        if self.print_counter >= 10:
            self.get_logger().info(
                f"State: {self.state} | "
                f"PreDist: {predock_dist:.3f} m | "
                f"e_long: {e_long:.3f} m | "
                f"e_lat: {e_lat:.3f} m | "
                f"e_yaw: {math.degrees(e_yaw):.2f} deg | "
                f"v: {self.v_cmd:.3f} w: {self.w_cmd:.3f} | "
                f"attempts: {self.attempts}/{self.max_attempts}"
            )
            self.print_counter = 0

        # ---------------- Publicar ----------------
        msg = Twist()
        msg.linear.x = self.v_cmd
        msg.angular.z = self.w_cmd
        self.cmd_vel_pub.publish(msg)

    # ------------------------------------------------------------------
    # Odometría
    # ------------------------------------------------------------------
    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = quat_to_yaw(msg.pose.pose.orientation)
        self.odom_received = True


def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )

    node = ControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_vel_pub.publish(Twist())
            node.get_logger().info('Robot stopped safely')
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()