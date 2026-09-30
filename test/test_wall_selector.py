from create3_dock_solution.perception.dock_model import (
    BoxValidationParams, DockGeometryParams, RansacParams,
)
from create3_dock_solution.perception.wall_selector import find_dock_wall
import numpy as np

_ANGLE_INC = 2 * np.pi / 720


def _side_wall_points(x=0.4, n=300, y_range=(-1.5, 1.5), noise=0.002, seed=10):
    """Pared lisa sin firma, cercana y con muchos más puntos que la del dock."""
    rng = np.random.default_rng(seed)
    y = np.linspace(y_range[0], y_range[1], n)
    xs = np.full_like(y, x) + rng.normal(0.0, noise, size=n)
    return np.stack([xs, y], axis=1)


def _dock_wall_points(
        y=1.2, gap=0.095, box=0.08, protrusion=0.08,
        x_half_extent=0.5, n_wall=80, noise=0.002, seed=11):
    """Pared perpendicular a la anterior, con la firma de las dos cajas."""
    rng = np.random.default_rng(seed)
    half_gap = gap / 2.0
    xs_wall = np.concatenate([
        np.linspace(half_gap + box, x_half_extent, n_wall // 2),
        np.linspace(-x_half_extent, -half_gap - box, n_wall // 2),
    ])
    ys_wall = np.full_like(xs_wall, y) + rng.normal(0.0, noise, size=xs_wall.shape)
    wall_pts = np.stack([xs_wall, ys_wall], axis=1)

    box_x = np.concatenate([
        np.linspace(half_gap, half_gap + box, 6),
        np.linspace(-half_gap - box, -half_gap, 6),
    ])
    box_y = np.full_like(box_x, y - protrusion)
    box_pts = np.stack([box_x, box_y], axis=1)

    return np.concatenate([wall_pts, box_pts], axis=0)


def test_find_dock_wall_prefers_wall_with_signature_over_larger_wall():
    """
    Reproduce el Escenario 1.

    Una pared lateral lisa (más inliers, más cercana) no debe ganarle a la
    pared del dock (con la firma) solo por tener más puntos.
    """
    points = np.concatenate([_side_wall_points(), _dock_wall_points()], axis=0)

    geom = DockGeometryParams()
    ransac_p = RansacParams()
    box_p = BoxValidationParams()
    rng = np.random.default_rng(42)

    result = find_dock_wall(
        points, geom, ransac_p, box_p, rng, _ANGLE_INC, ransac_p.max_wall_candidates)

    assert result is not None
    confidence, wall, coarse = result
    assert confidence > 0.0
    # la normal de la pared elegida debe apuntar casi puramente en -y (la del
    # dock, en y=1.2), no en -x (la pared lateral lisa, en x=0.4)
    assert wall.normal[1] < -0.9
    assert abs(coarse.pose.x) < 0.05
    assert abs(coarse.pose.y - 1.2) < 0.05


def test_find_dock_wall_returns_none_when_no_wall_has_the_signature():
    points = _side_wall_points()
    geom = DockGeometryParams()
    ransac_p = RansacParams()
    box_p = BoxValidationParams()
    rng = np.random.default_rng(43)

    result = find_dock_wall(
        points, geom, ransac_p, box_p, rng, _ANGLE_INC, ransac_p.max_wall_candidates)

    assert result is None
