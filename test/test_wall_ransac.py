from create3_dock_solution.perception.dock_model import RansacParams
from create3_dock_solution.perception.wall_ransac import fit_wall
import numpy as np


def _wall_points(n=200, y_range=(-0.4, 0.4), wall_x=1.2, noise=0.002, seed=0):
    rng = np.random.default_rng(seed)
    y = np.linspace(y_range[0], y_range[1], n)
    x = np.full_like(y, wall_x) + rng.normal(0.0, noise, size=n)
    return np.stack([x, y], axis=1)


def test_fit_wall_recovers_line_and_normal_towards_origin():
    points = _wall_points()
    wall = fit_wall(points, RansacParams(), np.random.default_rng(1))
    assert wall is not None
    assert wall.num_inliers > 150
    np.testing.assert_allclose(wall.point[0], 1.2, atol=0.01)
    assert wall.normal[0] < 0.0   # apunta hacia el origen (robot); la pared está en x>0


def test_fit_wall_rejects_sparse_cloud():
    points = np.random.default_rng(2).uniform(-0.05, 0.05, size=(5, 2))
    wall = fit_wall(points, RansacParams(min_inliers=15), np.random.default_rng(3))
    assert wall is None


def test_fit_wall_robust_to_outliers():
    points = _wall_points(n=150)
    outliers = np.random.default_rng(4).uniform(-2.0, 2.0, size=(40, 2))
    cloud = np.concatenate([points, outliers], axis=0)
    wall = fit_wall(cloud, RansacParams(), np.random.default_rng(5))
    assert wall is not None
    np.testing.assert_allclose(wall.point[0], 1.2, atol=0.03)
