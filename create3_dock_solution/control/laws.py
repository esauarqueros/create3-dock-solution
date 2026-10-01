"""
Leyes de control puras (sin estado) usadas por la máquina de estados de `controller.py`.

Todas devuelven `(v, w)` en m/s y rad/s, en el frame del robot.
"""
import math

from .params import AlignParams, ApproachParams, FinalParams, PolarParams
from ..perception.geometry import pose_relative, wrap_angle
from ..types import Pose2D


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def go_to_point(robot: Pose2D, target: Pose2D, p: ApproachParams) -> tuple[float, float]:
    """
    Modo `point_align` (B1): apuntar al punto y avanzar; girar en sitio si el rumbo es malo.

    No controla la orientación de llegada (eso lo hace ALIGN_AT_PREDOCK).
    """
    dx, dy = target.x - robot.x, target.y - robot.y
    dist = math.hypot(dx, dy)
    heading_err = wrap_angle(math.atan2(dy, dx) - robot.yaw)
    w = clamp(p.k_alpha * heading_err, -p.w_max, p.w_max)
    if abs(heading_err) > math.radians(p.rotate_in_place_deg):
        return 0.0, w
    v = clamp(p.k_v * dist * max(0.0, math.cos(heading_err)), p.v_min, p.v_max)
    # Cerca del punto no pasarse: la velocidad no supera 2*dist (m/s por m).
    v = min(v, max(2.0 * dist, 0.01))
    return v, w


def polar_to_pose(robot: Pose2D, target: Pose2D, ap: ApproachParams,
                  pp: PolarParams) -> tuple[float, float]:
    """
    Modo `polar` (B3, Astolfi): lleva el robot a la *pose* `target` (posición y yaw).

    rho = distancia, alpha = ángulo hacia el objetivo visto desde el robot,
    beta = orientación final pendiente. v se satura en [v_min_pass, v_max]
    para no frenar a cero en el pre-dock (se pasa de largo a FINAL).
    """
    r = pose_relative(target, robot)
    rho = math.hypot(r.x, r.y)
    alpha = wrap_angle(math.atan2(-r.y, -r.x) - r.yaw)
    beta = wrap_angle(-r.yaw - alpha)
    w = clamp(pp.k_alpha * alpha + pp.k_beta * beta, -ap.w_max, ap.w_max)
    if abs(alpha) > math.radians(ap.rotate_in_place_deg):
        return 0.0, clamp(ap.k_alpha * alpha, -ap.w_max, ap.w_max)
    v = clamp(pp.k_rho * rho, pp.v_min_pass, ap.v_max) * max(0.0, math.cos(alpha))
    return v, w


def rotate_to(yaw_err: float, p: AlignParams) -> float:
    """Giro en sitio para anular `yaw_err` = yaw_robot - yaw_objetivo (con piso w_min)."""
    w = -p.k_theta * yaw_err
    if w != 0.0 and abs(w) < p.w_min:
        w = math.copysign(p.w_min, w)
    return clamp(w, -p.w_max, p.w_max)


def axis_tracking(e_long: float, e_lat: float, e_yaw: float,
                  p: FinalParams) -> tuple[float, float]:
    """
    Tramo final (B2 tipo Stanley) sobre el eje del dock.

    El rumbo de referencia corta el eje con un ángulo `atan(k_lat * e_lat)`
    acotado a `max_lat_offset_deg`: corrige el error lateral de forma continua
    mientras avanza, sin saturaciones a mano con e_lat grande.
    """
    lat_offset = clamp(math.atan(p.k_lat * e_lat), -math.radians(p.max_lat_offset_deg),
                       math.radians(p.max_lat_offset_deg))
    heading_err = wrap_angle(-lat_offset - e_yaw)   # (yaw_goal - lat_offset) - yaw_robot
    w = clamp(p.k_yaw * heading_err, -p.w_max, p.w_max)
    v_nominal = p.v_slow if e_long > -p.slow_zone_m else p.v
    return v_nominal * max(0.0, math.cos(heading_err)), w


def rate_limit(target: float, previous: float, max_rate: float, dt: float) -> float:
    """Rampa de aceleración; `max_rate <= 0` la desactiva."""
    if max_rate <= 0.0:
        return target
    if dt <= 0.0:
        return previous
    step = max_rate * dt
    return clamp(target, previous - step, previous + step)
