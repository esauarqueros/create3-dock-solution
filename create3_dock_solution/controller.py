import math

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def wrap(angle):
    """Normaliza un ángulo a [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class ControlNode(Node):
    """
    Docking en 4 estados:

        GO_TO_PREDOCK -> ALIGN_AT_PREDOCK -> FINAL_APPROACH -> DOCKED

    El dock se define por una pose (target_x, target_y, target_theta).
    target_theta define un EJE: la recta que pasa por (target_x, target_y)
    con dirección target_theta. El pre-dock es un punto sobre ese eje,
    a d_pre metros "detrás" del dock. Desde ahí el robot avanza por el eje.
    """

    def __init__(self):
        super().__init__('control_node')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.timer = self.create_timer(0.1, self.control_loop)

        # ---------------- Pose (odometría) ----------------
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.odom_received = False

        # ---------------- Pose ficticia del dock ----------------
        self.target_x = 1.0
        self.target_y = 1.5
        self.target_theta = math.radians(0.0)  # rad

        # ---------------- Pre-dock (sobre el eje) ----------------
        self.d_pre = 0.50  # m detrás del dock, sobre el eje
        self.pre_x = self.target_x - self.d_pre * math.cos(self.target_theta)
        self.pre_y = self.target_y - self.d_pre * math.sin(self.target_theta)

        # ---------------- Parámetros GO_TO_PREDOCK ----------------
        self.k_v = 0.4
        self.k_alpha = 1.5
        self.v_max = 0.25
        self.v_min = 0.03
        self.w_max_go = 1.0
        self.predock_tol = 0.03            # m
        self.rotate_in_place_angle = math.radians(60.0)

        # ---------------- Parámetros ALIGN_AT_PREDOCK ----------------
        self.k_theta = 1.0
        self.w_max_align = 0.6
        self.w_min_align = 0.08            # vence fricción
        self.align_tol = math.radians(3.0)

        # ---------------- Parámetros FINAL_APPROACH ----------------
        self.v_dock = 0.03                 # m/s
        self.v_dock_slow = 0.015           # m/s cerca del dock
        self.slow_zone = 0.10              # m antes del dock
        self.dock_distance = 0.03          # m: parar cuando e_long >= -dock_distance
        self.k_lat = 5.0                   # 1/m  (Stanley: yaw_ref = theta - atan(k_lat*e_lat))
        self.max_lat_offset = math.radians(20.0)
        self.k_yaw = 1.5
        self.w_max_dock = 0.5
        # Umbrales de aborto -> volver a ALIGN_AT_PREDOCK
        self.abort_lat = 0.08              # m
        self.abort_yaw = math.radians(25.0)

        # ---------------- Estado ----------------
        self.state = "GO_TO_PREDOCK"
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.print_counter = 0

        self.get_logger().info(
            f'Control node started. Dock=({self.target_x:.2f}, {self.target_y:.2f}, '
            f'{math.degrees(self.target_theta):.0f} deg) | '
            f'Pre-dock=({self.pre_x:.2f}, {self.pre_y:.2f})'
        )

    # ------------------------------------------------------------------
    # Errores en el marco del dock
    # ------------------------------------------------------------------
    def dock_frame_errors(self):
        """
        Posición del robot expresada en el marco del dock
        (origen en el dock, eje x a lo largo de target_theta).

        e_long: negativo = aún no llega al dock (está "detrás"), 0 = en el dock
        e_lat : desviación lateral respecto al eje (positivo = izquierda del eje)
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
    # Lazo de control
    # ------------------------------------------------------------------
    def control_loop(self):
        if not self.odom_received:
            return

        e_long, e_lat, e_yaw = self.dock_frame_errors()

        # Error hacia el pre-dock (para GO_TO_PREDOCK)
        pdx = self.pre_x - self.robot_x
        pdy = self.pre_y - self.robot_y
        predock_dist = math.hypot(pdx, pdy)
        predock_heading_err = wrap(math.atan2(pdy, pdx) - self.robot_yaw)

        # Reset de comandos en cada ciclo (evita arrastrar valores viejos)
        self.v_cmd = 0.0
        self.w_cmd = 0.0

        # ---------------- GO_TO_PREDOCK ----------------
        if self.state == "GO_TO_PREDOCK":

            if predock_dist < self.predock_tol:
                self.state = "ALIGN_AT_PREDOCK"
                self.get_logger().info("Switching to ALIGN_AT_PREDOCK")
            else:
                self.w_cmd = clamp(
                    self.k_alpha * predock_heading_err,
                    -self.w_max_go, self.w_max_go
                )
                if abs(predock_heading_err) > self.rotate_in_place_angle:
                    # Muy desalineado: girar primero, no avanzar de lado
                    self.v_cmd = 0.0
                else:
                    v = self.k_v * predock_dist * max(0.0, math.cos(predock_heading_err))
                    self.v_cmd = clamp(v, self.v_min, self.v_max)
                    # No pasarse del objetivo con el mínimo
                    self.v_cmd = min(self.v_cmd, max(predock_dist * 2.0, 0.01))

        # ---------------- ALIGN_AT_PREDOCK ----------------
        elif self.state == "ALIGN_AT_PREDOCK":

            if abs(e_lat) > self.abort_lat:
                # Nos movimos del eje: volver a posicionarse
                self.get_logger().warn("Off axis at pre-dock, back to GO_TO_PREDOCK")
                self.state = "GO_TO_PREDOCK"
            elif abs(e_yaw) < self.align_tol:
                self.state = "FINAL_APPROACH"
                self.get_logger().info("Switching to FINAL_APPROACH")
            else:
                w = self.k_theta * e_yaw
                # Velocidad angular mínima para vencer fricción
                if abs(w) < self.w_min_align:
                    w = math.copysign(self.w_min_align, e_yaw)
                self.w_cmd = clamp(w, -self.w_max_align, self.w_max_align)

        # ---------------- FINAL_APPROACH ----------------
        elif self.state == "FINAL_APPROACH":

            if e_long >= -self.dock_distance:
                self.state = "DOCKED"
                self.get_logger().info(
                    f"DOCKED | e_long={e_long:.3f} m, e_lat={e_lat:.3f} m, "
                    f"e_yaw={math.degrees(e_yaw):.2f} deg"
                )
            elif abs(e_lat) > self.abort_lat or abs(e_yaw) > self.abort_yaw:
                self.get_logger().warn(
                    f"Abort final approach (e_lat={e_lat:.3f} m, "
                    f"e_yaw={math.degrees(e_yaw):.1f} deg) -> ALIGN_AT_PREDOCK"
                )
                self.state = "GO_TO_PREDOCK"
            else:
                # Stanley: yaw de referencia = eje del dock corregido por error lateral.
                # e_lat > 0 (izquierda del eje) -> apuntar un poco a la derecha.
                lat_offset = clamp(
                    math.atan(self.k_lat * e_lat),
                    -self.max_lat_offset, self.max_lat_offset
                )
                yaw_ref = self.target_theta - lat_offset
                yaw_err_eff = wrap(yaw_ref - self.robot_yaw)

                self.w_cmd = clamp(
                    self.k_yaw * yaw_err_eff, -self.w_max_dock, self.w_max_dock
                )

                # Velocidad lenta, reducida si el yaw efectivo está mal
                v = self.v_dock_slow if e_long > -self.slow_zone else self.v_dock
                self.v_cmd = v * max(0.0, math.cos(yaw_err_eff))

        # ---------------- DOCKED ----------------
        elif self.state == "DOCKED":
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
                f"v: {self.v_cmd:.3f} w: {self.w_cmd:.3f}"
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

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
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