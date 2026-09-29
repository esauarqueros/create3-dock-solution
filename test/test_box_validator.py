from create3_dock_solution.perception.box_validator import (
    cluster_points, coarse_confidence, extract_protrusion_points, validate_boxes,
)
from create3_dock_solution.perception.dock_model import (
    BoxValidationParams, DockGeometryParams, RansacParams,
)
from create3_dock_solution.perception.wall_ransac import WallLine
import numpy as np


def _synthetic_wall_and_boxes(gap=0.095, box=0.08, protrusion=0.08, wall_x=1.0):
    wall = WallLine(point=np.array([wall_x, 0.0]), normal=np.array([-1.0, 0.0]),
                    inlier_ratio=0.8, num_inliers=100)
    ys_left = np.linspace(gap / 2, gap / 2 + box, 6)
    ys_right = np.linspace(-gap / 2 - box, -gap / 2, 6)
    pts = [(wall_x - protrusion, y) for y in np.concatenate([ys_left, ys_right])]
    return wall, np.array(pts)


def test_validate_boxes_accepts_correct_geometry():
    wall, points = _synthetic_wall_and_boxes()
    geom = DockGeometryParams()
    box_p = BoxValidationParams()

    tangential, perp = extract_protrusion_points(points, wall, box_p)
    clusters = cluster_points(tangential, box_p)
    assert len(clusters) == 2

    coarse = validate_boxes(clusters, tangential, perp, wall, geom, box_p)
    assert coarse is not None
    assert coarse.gap_error < box_p.gap_tolerance

    confidence = coarse_confidence(coarse, wall, RansacParams(), box_p)
    assert confidence > 0.5


def test_validate_boxes_rejects_wrong_gap():
    wall, points = _synthetic_wall_and_boxes(gap=0.30)   # hueco muy grande: no es el dock
    geom = DockGeometryParams()
    box_p = BoxValidationParams()

    tangential, perp = extract_protrusion_points(points, wall, box_p)
    clusters = cluster_points(tangential, box_p)
    coarse = validate_boxes(clusters, tangential, perp, wall, geom, box_p)
    assert coarse is None


def test_validate_boxes_rejects_single_box():
    """Caso del robot a un costado del dock: solo una caja visible -> sin detección."""
    wall, points = _synthetic_wall_and_boxes()
    only_one_box = points[points[:, 1] > 0]
    geom = DockGeometryParams()
    box_p = BoxValidationParams()

    tangential, perp = extract_protrusion_points(only_one_box, wall, box_p)
    clusters = cluster_points(tangential, box_p)
    coarse = validate_boxes(clusters, tangential, perp, wall, geom, box_p)
    assert coarse is None
