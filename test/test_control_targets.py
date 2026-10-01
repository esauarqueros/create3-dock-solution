import math

from create3_dock_solution.control.dock_targets import (
    axis_errors, dock_goal_from_marker, predock_from_goal,
)
from create3_dock_solution.perception.geometry import pose_compose, pose_relative
from create3_dock_solution.types import Pose2D
import pytest


def test_goal_is_offset_along_wall_normal_and_faces_the_wall():
    marker = Pose2D(1.95, 0.1, math.pi)     # normal de la pared hacia -x (hacia la sala)
    goal = dock_goal_from_marker(marker, 0.266)
    assert goal.x == pytest.approx(1.95 - 0.266)
    assert goal.y == pytest.approx(0.1)
    assert abs(goal.yaw) == pytest.approx(0.0, abs=1e-9)   # mira a +x, hacia la pared


def test_predock_is_behind_goal_on_the_axis():
    marker = Pose2D(0.0, 0.0, math.pi / 2)  # pared horizontal, normal hacia +y
    goal = dock_goal_from_marker(marker, 0.266)
    predock = predock_from_goal(goal, 0.40)
    assert predock.x == pytest.approx(0.0, abs=1e-9)
    assert predock.y == pytest.approx(0.666)
    assert predock.yaw == pytest.approx(goal.yaw)


def test_axis_errors_sign_convention():
    goal = Pose2D(1.0, 0.0, 0.0)            # pared en +x
    errors = axis_errors(goal, Pose2D(0.5, 0.02, math.radians(5.0)))
    assert errors.e_long == pytest.approx(-0.5)            # antes del dock
    assert errors.e_lat == pytest.approx(0.02)             # a la izquierda del eje
    assert math.degrees(errors.e_yaw) == pytest.approx(5.0)


def test_pose_relative_is_inverse_of_pose_compose():
    ref = Pose2D(0.3, -1.2, 2.1)
    child = Pose2D(0.4, 0.25, -0.7)
    back = pose_relative(ref, pose_compose(ref, child))
    assert back.x == pytest.approx(child.x)
    assert back.y == pytest.approx(child.y)
    assert back.yaw == pytest.approx(child.yaw)
