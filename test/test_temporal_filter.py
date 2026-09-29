from create3_dock_solution.perception.dock_model import FilterParams
from create3_dock_solution.perception.temporal_filter import DockPoseFilter
from create3_dock_solution.types import Pose2D


def test_filter_converges_towards_repeated_measurement():
    dock_filter = DockPoseFilter(FilterParams())
    pose = Pose2D(1.0, 0.1, 0.2)

    estimate = None
    for i in range(20):
        estimate = dock_filter.update(pose, confidence=0.9, stamp=float(i) * 0.1)

    assert estimate is not None
    assert abs(estimate.pose.x - 1.0) < 0.01
    assert abs(estimate.pose.y - 0.1) < 0.01


def test_filter_returns_none_after_lost_timeout():
    dock_filter = DockPoseFilter(FilterParams(lost_timeout_s=0.5))
    dock_filter.update(Pose2D(1.0, 0.0, 0.0), confidence=0.9, stamp=0.0)

    estimate = dock_filter.update(None, confidence=0.0, stamp=1.0)

    assert estimate is None


def test_filter_rejects_spurious_jump():
    dock_filter = DockPoseFilter(FilterParams(max_jump_m=0.3))
    for i in range(10):
        dock_filter.update(Pose2D(1.0, 0.0, 0.0), confidence=0.9, stamp=float(i) * 0.1)

    estimate = dock_filter.update(Pose2D(5.0, 5.0, 0.0), confidence=0.9, stamp=1.0)

    assert estimate is not None
    assert abs(estimate.pose.x - 1.0) < 0.05
