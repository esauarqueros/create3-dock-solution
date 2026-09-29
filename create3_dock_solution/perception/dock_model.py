"""
Dataclasses de parámetros del detector + geometría/plantilla del dock.

Cada dataclass mapea 1:1 a un grupo de `config/dock_detector.yaml`. Los valores
por defecto son los mismos que ese archivo, para que el detector funcione
razonablemente incluso si se instancia con `params={}` (p.ej. en tests).
"""
from dataclasses import dataclass, fields

import numpy as np


def _from_dict(cls, data: dict):
    data = data or {}
    known = {f.name for f in fields(cls)}
    kwargs = {k: v for k, v in data.items() if k in known}
    return cls(**kwargs)


@dataclass
class DockGeometryParams:
    box_size: float = 0.08
    box_gap: float = 0.095
    box_protrusion: float = 0.08
    wall_search_range_max: float = 3.0
    wall_half_extent: float = 0.35
    template_point_spacing: float = 0.01

    @classmethod
    def from_dict(cls, data: dict):
        return _from_dict(cls, data)


@dataclass
class RansacParams:
    max_iterations: int = 200
    inlier_threshold: float = 0.015
    min_inlier_ratio: float = 0.35
    min_inliers: int = 15

    @classmethod
    def from_dict(cls, data: dict):
        return _from_dict(cls, data)


@dataclass
class BoxValidationParams:
    gap_tolerance: float = 0.02
    protrusion_tolerance: float = 0.02
    width_tolerance: float = 0.02
    cluster_max_gap: float = 0.03
    cluster_min_points: int = 2
    min_protrusion_m: float = 0.03
    max_protrusion_m: float = 0.15

    @classmethod
    def from_dict(cls, data: dict):
        return _from_dict(cls, data)


@dataclass
class IcpParams:
    enable: bool = True
    coarse_to_fine_range_m: float = 1.0
    max_iterations: int = 25
    max_correspondence_dist: float = 0.05
    convergence_translation_eps: float = 0.002
    convergence_rotation_eps_rad: float = 0.01
    min_correspondences: int = 10
    max_residual_rms: float = 0.02
    max_translation_correction_m: float = 0.15
    max_rotation_correction_rad: float = 0.52  # ~30 grados

    @classmethod
    def from_dict(cls, data: dict):
        return _from_dict(cls, data)


@dataclass
class FilterParams:
    alpha_min: float = 0.05
    alpha_max: float = 0.6
    confidence_decay_per_s: float = 0.5
    lost_timeout_s: float = 1.5
    max_jump_m: float = 0.5
    outlier_reject_confidence: float = 0.3

    @classmethod
    def from_dict(cls, data: dict):
        return _from_dict(cls, data)


def build_template(geom: DockGeometryParams) -> np.ndarray:
    """
    Nube de puntos (N,2) del modelo "pared + 2 cajas" en el frame local del dock.

    Origen en el centro del hueco sobre la pared, +x = normal de la pared (hacia
    la sala), +y = a lo largo de la pared. Usada como plantilla del ICP (A3).
    """
    half_gap = geom.box_gap / 2.0
    points = []

    y_wall = np.arange(
        half_gap + geom.box_size, geom.wall_half_extent, geom.template_point_spacing)
    for y in y_wall:
        points.append((0.0, y))
        points.append((0.0, -y))

    for sign in (1.0, -1.0):
        y_inner = sign * half_gap
        for step in np.arange(0.0, geom.box_size, geom.template_point_spacing):
            points.append((geom.box_protrusion, y_inner + sign * step))   # cara frontal
        y_outer = sign * (half_gap + geom.box_size)
        for x in np.arange(0.0, geom.box_protrusion, geom.template_point_spacing):
            points.append((x, y_inner))    # cara lateral interior (hacia el hueco)
            points.append((x, y_outer))    # cara lateral exterior

    return np.array(points, dtype=float)
