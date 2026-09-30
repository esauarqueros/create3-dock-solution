from create3_dock_solution.perception.geometry import deskew_points, integrate_unicycle
from create3_dock_solution.perception.odom_history import OdomHistory
from create3_dock_solution.types import Pose2D
import numpy as np
import pytest


def test_pose_at_interpolates_between_samples():
    history = OdomHistory()
    history.add(0.0, Pose2D(0.0, 0.0, 0.0), 0.0, 0.0)
    history.add(0.1, Pose2D(0.1, 0.2, 0.4), 0.0, 0.0)

    pose = history.pose_at(0.025)

    assert pose.x == pytest.approx(0.025)
    assert pose.y == pytest.approx(0.05)
    assert pose.yaw == pytest.approx(0.1)


def test_pose_at_interpolates_yaw_across_pi():
    history = OdomHistory()
    history.add(0.0, Pose2D(0.0, 0.0, np.pi - 0.1), 0.0, 0.0)
    history.add(0.1, Pose2D(0.0, 0.0, -np.pi + 0.1), 0.0, 0.0)

    pose = history.pose_at(0.05)

    assert abs(abs(pose.yaw) - np.pi) < 1e-6   # pasa por ±pi, no por 0


def test_pose_at_extrapolates_only_a_short_time():
    history = OdomHistory(max_extrapolation_s=0.1)
    history.add(0.0, Pose2D(0.0, 0.0, 0.0), 0.2, 0.0)

    ahead = history.pose_at(0.05)
    assert ahead.x == pytest.approx(0.01)
    assert history.pose_at(0.5) is None


def test_pose_at_returns_none_for_stamps_older_than_history():
    history = OdomHistory(max_age_s=1.0)
    for i in range(30):
        history.add(0.1 * i, Pose2D(0.0, 0.0, 0.0), 0.0, 0.0)

    assert history.pose_at(0.5) is None
    assert history.pose_at(2.5) is not None


def test_deskew_undoes_rotation_during_sweep():
    # Un punto fijo en el mundo, visto por un robot que gira mientras barre.
    v, w = 0.1, 1.0
    world_point = np.array([1.5, 0.3])
    beam_times = np.array([0.0, 0.03, 0.06, 0.09])
    observed = []
    for t in beam_times:
        pose = integrate_unicycle(Pose2D(0.0, 0.0, 0.0), v, w, t)
        c, s = np.cos(pose.yaw), np.sin(pose.yaw)
        rel = world_point - np.array([pose.x, pose.y])
        observed.append([c * rel[0] + s * rel[1], -s * rel[0] + c * rel[1]])

    corrected = deskew_points(np.array(observed), beam_times, v, w)

    assert np.allclose(corrected, world_point, atol=1e-9)
