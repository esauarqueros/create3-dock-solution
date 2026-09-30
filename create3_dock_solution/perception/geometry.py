"""
Utilidades geométricas puras (numpy) compartidas por el detector.

Convención de `Pose2D` usada en todo `perception/`: describe la pose de un frame
hijo (p.ej. el láser, o el dock) vista desde un frame padre (p.ej. base_link, u
odom) -- igual que una transformada de TF o el campo `pose` de `nav_msgs/Odometry`.
"""
import numpy as np

from ..types import Pose2D


def wrap_angle(angle: float) -> float:
    return float((angle + np.pi) % (2 * np.pi) - np.pi)


def valid_range_mask(ranges: np.ndarray, range_min: float, range_max: float) -> np.ndarray:
    """Máscara de las lecturas finitas y dentro de [range_min, range_max]."""
    ranges = np.asarray(ranges, dtype=float)
    return np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)


def polar_to_points(
        ranges: np.ndarray, angle_min: float, angle_inc: float,
        range_min: float, range_max: float) -> np.ndarray:
    """
    Convierte un scan polar (rangos) a puntos (N,2) en el frame del láser.

    Descarta lecturas inválidas o fuera de rango.
    """
    ranges = np.asarray(ranges, dtype=float)
    angles = angle_min + np.arange(ranges.shape[0]) * angle_inc
    valid = valid_range_mask(ranges, range_min, range_max)
    r = ranges[valid]
    a = angles[valid]
    return np.stack([r * np.cos(a), r * np.sin(a)], axis=1)


def pose_to_matrix(pose: Pose2D):
    c, s = np.cos(pose.yaw), np.sin(pose.yaw)
    r = np.array([[c, -s], [s, c]])
    t = np.array([pose.x, pose.y])
    return r, t


def matrix_to_pose(r: np.ndarray, t: np.ndarray) -> Pose2D:
    yaw = float(np.arctan2(r[1, 0], r[0, 0]))
    return Pose2D(x=float(t[0]), y=float(t[1]), yaw=yaw)


def transform_points(points: np.ndarray, pose: Pose2D) -> np.ndarray:
    """Transforma puntos expresados en el frame local de `pose` hacia el frame padre."""
    r, t = pose_to_matrix(pose)
    return points @ r.T + t


def rotate90(v: np.ndarray) -> np.ndarray:
    return np.array([-v[1], v[0]])


def pose_compose(base_to_parent: Pose2D, child_to_base: Pose2D) -> Pose2D:
    """
    Compone dos poses encadenadas (p.ej. dock-en-base_link con base_link-en-odom).

    Devuelve `child` (el dock) expresado en el frame padre (odom).
    """
    r, t = pose_to_matrix(base_to_parent)
    point = r @ np.array([child_to_base.x, child_to_base.y]) + t
    yaw = wrap_angle(base_to_parent.yaw + child_to_base.yaw)
    return Pose2D(x=float(point[0]), y=float(point[1]), yaw=yaw)


def _unicycle_displacement(v: float, w: float, dt: np.ndarray):
    """Desplazamiento (dx, dy, dyaw) de un uniciclo a (v, w) constantes, en su frame inicial."""
    dt = np.asarray(dt, dtype=float)
    dyaw = w * dt
    if abs(w) > 1e-6:
        dx = (v / w) * np.sin(dyaw)
        dy = (v / w) * (1.0 - np.cos(dyaw))
    else:
        dx = v * dt
        dy = np.zeros_like(dt)
    return dx, dy, dyaw


def integrate_unicycle(pose: Pose2D, v: float, w: float, dt: float) -> Pose2D:
    """Avanza `pose` durante `dt` con velocidades de cuerpo (v, w) constantes."""
    dx, dy, dyaw = _unicycle_displacement(v, w, dt)
    step = Pose2D(x=float(dx), y=float(dy), yaw=float(dyaw))
    return pose_compose(pose, step)


def deskew_points(points: np.ndarray, beam_times: np.ndarray, v: float, w: float) -> np.ndarray:
    """
    Corrige la distorsión por movimiento durante el barrido del LIDAR.

    Cada punto `points[i]` está expresado en `base_link` en el instante
    `beam_times[i]` (segundos desde el primer haz, que es el `stamp` del scan);
    se devuelve expresado en `base_link` en el instante del primer haz,
    asumiendo (v, w) constantes durante el barrido.
    """
    if points.shape[0] == 0:
        return points
    dx, dy, dyaw = _unicycle_displacement(v, w, beam_times)
    c, s = np.cos(dyaw), np.sin(dyaw)
    x = c * points[:, 0] - s * points[:, 1] + dx
    y = s * points[:, 0] + c * points[:, 1] + dy
    return np.stack([x, y], axis=1)
