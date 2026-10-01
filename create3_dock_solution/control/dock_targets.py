"""
Del marcador que entrega el detector al dock real y al pre-dock.

Ver docs/CONTROLADOR_IMPLEMENTADO.md.

Marcador `m` (DockEstimate.pose): centro del hueco entre las cajas, sobre la
pared; yaw = normal de la pared hacia la sala. Con `n = (cos m.yaw, sin m.yaw)`:

- dock real (goal): `m + dock_offset_m * n`, yaw `m.yaw + pi` (el robot acoplado mira a la pared).
- pre-dock:         `goal + predock_distance_m * n`, mismo yaw que el goal.

Errores en el frame del goal (x hacia la pared): `e_long < 0` antes del dock,
`e_lat > 0` robot a la izquierda del eje, `e_yaw = yaw_robot - yaw_goal`.
"""
from dataclasses import dataclass
import math

from ..perception.geometry import pose_relative, wrap_angle
from ..types import Pose2D


@dataclass
class AxisErrors:
    e_long: float
    e_lat: float
    e_yaw: float


def dock_goal_from_marker(marker: Pose2D, dock_offset_m: float) -> Pose2D:
    c, s = math.cos(marker.yaw), math.sin(marker.yaw)
    return Pose2D(marker.x + dock_offset_m * c, marker.y + dock_offset_m * s,
                  wrap_angle(marker.yaw + math.pi))


def predock_from_goal(goal: Pose2D, predock_distance_m: float) -> Pose2D:
    # goal.yaw apunta a la pared: el pre-dock queda "detrás", hacia la sala.
    c, s = math.cos(goal.yaw), math.sin(goal.yaw)
    return Pose2D(goal.x - predock_distance_m * c, goal.y - predock_distance_m * s, goal.yaw)


def axis_errors(goal: Pose2D, robot: Pose2D) -> AxisErrors:
    r = pose_relative(goal, robot)
    return AxisErrors(e_long=r.x, e_lat=r.y, e_yaw=r.yaw)
