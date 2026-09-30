from create3_dock_solution.perception.dock_model import (
    build_template, DockGeometryParams, IcpParams,
)
from create3_dock_solution.perception.icp_refiner import is_refinement_plausible, refine
from create3_dock_solution.types import Pose2D
import numpy as np
import pytest


def _transform(points, pose: Pose2D):
    c, s = np.cos(pose.yaw), np.sin(pose.yaw)
    r = np.array([[c, -s], [s, c]])
    return points @ r.T + np.array([pose.x, pose.y])


def test_refine_recovers_known_pose():
    geom = DockGeometryParams()
    template = build_template(geom)
    true_pose = Pose2D(x=1.0, y=0.05, yaw=np.deg2rad(8.0))
    rng = np.random.default_rng(0)
    scan = _transform(template, true_pose) + rng.normal(0.0, 0.002, size=template.shape)
    # Estimación gruesa (coarse) típica: validada geométricamente por box_validator
    # dentro de sus tolerancias (~2cm/±unos grados), no una adivinanza arbitraria --
    # el ICP es un refinamiento, no una búsqueda global.
    init_pose = Pose2D(x=0.98, y=0.03, yaw=np.deg2rad(4.0))

    params = IcpParams(estimate_yaw=True)
    result = refine(scan, template, init_pose, params)

    assert result is not None
    assert abs(result.pose.x - true_pose.x) < 0.01
    assert abs(result.pose.y - true_pose.y) < 0.01
    assert abs(result.pose.yaw - true_pose.yaw) < np.deg2rad(3.0)
    assert is_refinement_plausible(init_pose, result.pose, params)


def test_refine_translation_only_keeps_initial_yaw():
    """Por defecto el ICP solo corrige posición: el yaw viene de la pared."""
    geom = DockGeometryParams()
    template = build_template(geom)
    true_pose = Pose2D(x=0.8, y=-0.03, yaw=np.deg2rad(-12.0))
    rng = np.random.default_rng(2)
    scan = _transform(template, true_pose) + rng.normal(0.0, 0.002, size=template.shape)
    init_pose = Pose2D(x=0.78, y=-0.01, yaw=true_pose.yaw)

    result = refine(scan, template, init_pose, IcpParams())

    assert result is not None
    assert abs(result.pose.x - true_pose.x) < 0.005
    assert abs(result.pose.y - true_pose.y) < 0.005
    assert result.pose.yaw == pytest.approx(init_pose.yaw, abs=1e-9)


def test_refine_fails_explicitly_with_too_few_points():
    geom = DockGeometryParams()
    template = build_template(geom)
    scan = np.random.default_rng(1).uniform(-0.05, 0.05, size=(3, 2))

    result = refine(scan, template, Pose2D(0.9, 0.0, 0.0), IcpParams(min_correspondences=10))

    assert result is None
