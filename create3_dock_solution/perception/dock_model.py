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
    # Bajo a propósito: con varias paredes a la vista ninguna concentra >35% de
    # los puntos; quien discrimina la pared del dock es la firma de las cajas.
    min_inlier_ratio: float = 0.10
    min_inliers: int = 15
    max_wall_candidates: int = 3
    # Referencia para el término de inliers de coarse_confidence (desacoplado
    # del umbral de aceptación de arriba para no inflar la confianza).
    confidence_inlier_ratio_ref: float = 0.35

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
    # Tolerancias/confianza adaptativas por rango (ver docs/ESTRATEGIA.md):
    # a mayor distancia, menos puntos LIDAR caen sobre cada caja (resolución
    # angular fija), así que las tolerancias fijas de arriba se usan como piso
    # y se relajan proporcionalmente a angle_inc * distancia más allá de él.
    gap_tolerance_range_factor: float = 2.5
    width_tolerance_range_factor: float = 2.0
    protrusion_tolerance_range_factor: float = 1.5
    incidence_factor: float = 2.0
    cluster_min_points_far: int = 1
    cluster_min_points_far_range_m: float = 1.8

    @classmethod
    def from_dict(cls, data: dict):
        return _from_dict(cls, data)


@dataclass
class IcpParams:
    enable: bool = True
    coarse_to_fine_range_m: float = 1.0
    # False: el ICP solo corrige la posición y se conserva el yaw de la pared.
    estimate_yaw: bool = False
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
    # Re-enganche: N mediciones seguidas rechazadas por salto pero a menos de
    # reinit_consistency_m entre sí reemplazan la pose filtrada.
    reinit_after_consecutive: int = 3
    reinit_consistency_m: float = 0.10

    @classmethod
    def from_dict(cls, data: dict):
        return _from_dict(cls, data)


@dataclass
class DeskewParams:
    # Corrige el movimiento del robot durante el barrido (~0.1 s) del LIDAR
    # real. No hace nada si el scan trae time_increment == 0 (Gazebo).
    enable: bool = True

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
