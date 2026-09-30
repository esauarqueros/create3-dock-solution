from create3_dock_solution.perception.box_validator import (
    adapt_box_params, cluster_points, coarse_confidence, CoarseDetection,
    extract_protrusion_points, validate_boxes,
)
from create3_dock_solution.perception.dock_model import (
    BoxValidationParams, DockGeometryParams, RansacParams,
)
from create3_dock_solution.perception.wall_ransac import WallLine
from create3_dock_solution.types import Pose2D
import numpy as np
import pytest

_ANGLE_INC = 2 * np.pi / 720


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
    assert coarse.support_points == 12

    distance = float(wall.point[0])
    confidence = coarse_confidence(coarse, wall, RansacParams(), box_p, geom, _ANGLE_INC, distance)
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


def test_adapt_box_params_matches_its_own_formula_at_several_ranges():
    base = BoxValidationParams()
    for distance in (0.5, 1.0, 1.5, 2.0, 3.0):
        adapted = adapt_box_params(base, distance, _ANGLE_INC)
        spacing = _ANGLE_INC * distance
        assert adapted.gap_tolerance == pytest.approx(
            max(base.gap_tolerance, base.gap_tolerance_range_factor * spacing))
        assert adapted.width_tolerance == pytest.approx(
            max(base.width_tolerance, base.width_tolerance_range_factor * spacing))
        assert adapted.protrusion_tolerance == pytest.approx(
            max(base.protrusion_tolerance, base.protrusion_tolerance_range_factor * spacing))

    assert adapt_box_params(base, 1.0, _ANGLE_INC).cluster_min_points == base.cluster_min_points
    far = adapt_box_params(base, base.cluster_min_points_far_range_m + 0.5, _ANGLE_INC)
    assert far.cluster_min_points == base.cluster_min_points_far


def test_validate_boxes_ignores_width_check_for_single_point_cluster():
    """
    Un lado con un solo punto no puede medir su propio ancho.

    No debe bloquear la detección si el hueco y la protrusión sí calzan
    (ver docs/ESTRATEGIA.md, 9.2).
    """
    wall, points = _synthetic_wall_and_boxes()
    right_mask = points[:, 1] < 0
    # el último punto de la derecha (linspace creciente) es el borde interior,
    # el más cercano al hueco -- el relevante para un muestreo disperso realista
    sparse_right = points[right_mask][-1:]
    sparse_points = np.concatenate([points[~right_mask], sparse_right], axis=0)

    geom = DockGeometryParams()
    box_p = BoxValidationParams(cluster_min_points=1)

    tangential, perp = extract_protrusion_points(sparse_points, wall, box_p)
    clusters = cluster_points(tangential, box_p)
    assert len(clusters) == 2
    assert min(len(c) for c in clusters) == 1

    coarse = validate_boxes(clusters, tangential, perp, wall, geom, box_p)
    assert coarse is not None
    assert coarse.support_points == 7


def test_coarse_confidence_decreases_with_sparser_coverage_at_range():
    wall = WallLine(
        point=np.array([1.0, 0.0]), normal=np.array([-1.0, 0.0]),
        inlier_ratio=0.35, num_inliers=100)
    geom = DockGeometryParams()
    box_p = BoxValidationParams()
    ransac_p = RansacParams(confidence_inlier_ratio_ref=0.35)

    def confidence_for(support_points: int, distance: float) -> float:
        coarse = CoarseDetection(
            pose=Pose2D(1.0, 0.0, 0.0), gap_error=0.0, width_error=0.0,
            protrusion_error=0.0, support_points=support_points)
        return coarse_confidence(coarse, wall, ransac_p, box_p, geom, _ANGLE_INC, distance)

    # A la misma geometría (sin error), menos puntos de soporte relativo a lo
    # esperado a esa distancia -> menor confianza.
    assert confidence_for(8, 1.0) > confidence_for(2, 3.0)
