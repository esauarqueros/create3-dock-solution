from create3_dock_solution.detector import DockDetector
from create3_dock_solution.types import Pose2D
import numpy as np


def _synthetic_ranges(
        wall_x=1.2, n=720, angle_min=-np.pi, angle_inc=2 * np.pi / 720,
        box_gap=0.095, box_size=0.08, box_protrusion=0.08, max_range=8.0,
        noise=0.001, seed=0, visible_boxes=('left', 'right')):
    """
    Genera un LaserScan sintético de un robot en base_link viendo la pared del dock.

    Robot en el origen mirando +x, pared en x=wall_x con las cajas de la geometría
    del dock sobresaliendo de ella. `visible_boxes` controla si se renderizan una o
    las dos cajas -- con una sola (p.ej. porque la otra queda oculta por el ángulo
    rasante de vista, como cuando el robot arranca muy a un costado del dock) el
    rayo simplemente sigue hasta la pared llana detrás, tal como lo vería el LIDAR
    real en ese caso.
    """
    rng = np.random.default_rng(seed)
    angles = angle_min + np.arange(n) * angle_inc
    ranges = np.full(n, max_range)
    half_gap = box_gap / 2.0

    for i, a in enumerate(angles):
        if abs(a) > np.deg2rad(60):
            continue
        d = np.cos(a)
        if abs(d) < 1e-6:
            continue
        y_at_wall = wall_x * np.tan(a)
        side = 'left' if y_at_wall > 0 else 'right'
        on_box = half_gap < abs(y_at_wall) < half_gap + box_size and side in visible_boxes
        r = (wall_x - box_protrusion) / d if on_box else wall_x / d
        ranges[i] = max(0.05, r + rng.normal(0.0, noise))

    return ranges, angle_min, angle_inc


def test_detector_detects_dock_when_facing_it():
    ranges, angle_min, angle_inc = _synthetic_ranges(wall_x=1.2)
    detector = DockDetector(params={})
    robot_pose = Pose2D(0.0, 0.0, 0.0)

    estimate = None
    for i in range(5):
        estimate = detector.update(ranges, angle_min, angle_inc, robot_pose, stamp=float(i) * 0.1)

    assert estimate is not None
    assert estimate.confidence > 0.3
    assert abs(estimate.pose.x - 1.2) < 0.05
    assert abs(estimate.pose.y) < 0.05


def test_detector_returns_none_or_low_confidence_from_the_side():
    """
    Robot a un costado del dock: por el ángulo rasante solo se ve una caja.

    Sin la geometría completa (hueco + ambas cajas) no hay detección confiable.
    Este es el caso que le interesa a la recuperación del controller (ir al
    centro de la sala para mejorar el ángulo, ver docs/ESTRATEGIA.md).
    """
    ranges, angle_min, angle_inc = _synthetic_ranges(wall_x=1.2, visible_boxes=('left',))
    detector = DockDetector(params={})
    robot_pose = Pose2D(0.0, 0.0, 0.0)

    estimate = None
    for i in range(5):
        estimate = detector.update(ranges, angle_min, angle_inc, robot_pose, stamp=float(i) * 0.1)

    assert estimate is None or estimate.confidence < 0.3
