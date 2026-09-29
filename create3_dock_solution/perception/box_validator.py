"""
A1 usado como capa de validación geométrica (no como pipeline alterno).

Dado el ajuste de pared (A2), busca puntos que sobresalen de ella, los agrupa
en clusters y valida que un par de clusters tenga el hueco (~9.5cm) y ancho
(~8cm) esperados de las dos cajas del dock.
"""
from dataclasses import dataclass

import numpy as np

from .dock_model import BoxValidationParams, DockGeometryParams, RansacParams
from .geometry import rotate90
from .wall_ransac import WallLine
from ..types import Pose2D


@dataclass
class CoarseDetection:
    # pose: dock en base_link, origen en el centro del hueco, +x = normal de la pared
    pose: Pose2D
    gap_error: float
    width_error: float
    protrusion_error: float


def extract_protrusion_points(points: np.ndarray, wall: WallLine, params: BoxValidationParams):
    """
    Filtra los puntos que sobresalen de la pared una cantidad de las cajas.

    Devuelve (tangencial, perpendicular) de esos puntos.
    """
    rel = points - wall.point
    perp = rel @ wall.normal
    t_axis = rotate90(wall.normal)
    tangential = rel @ t_axis
    mask = (perp > params.min_protrusion_m) & (perp < params.max_protrusion_m)
    return tangential[mask], perp[mask]


def cluster_points(tangential: np.ndarray, params: BoxValidationParams):
    """
    Agrupa índices de `tangential` en clusters contiguos.

    El gap máximo entre puntos consecutivos es `cluster_max_gap`. Devuelve una
    lista de arrays de índices.
    """
    n = tangential.shape[0]
    if n == 0:
        return []
    order = np.argsort(tangential)
    clusters = []
    start = 0
    for i in range(1, n):
        if tangential[order[i]] - tangential[order[i - 1]] > params.cluster_max_gap:
            clusters.append(order[start:i])
            start = i
    clusters.append(order[start:n])
    return [c for c in clusters if len(c) >= params.cluster_min_points]


def _cluster_summary(tangential: np.ndarray, perp: np.ndarray, idx: np.ndarray) -> dict:
    t = tangential[idx]
    return {
        't_min': float(t.min()),
        't_max': float(t.max()),
        't_mid': float(t.mean()),
        'width': float(t.max() - t.min()),
        'perp_mean': float(perp[idx].mean()),
    }


def _pair_errors(left: dict, right: dict, geom: DockGeometryParams) -> tuple[float, float, float]:
    gap = right['t_min'] - left['t_max']
    gap_error = abs(gap - geom.box_gap)
    width_error = max(
        abs(left['width'] - geom.box_size),
        abs(right['width'] - geom.box_size),
    )
    protrusion_error = max(
        abs(left['perp_mean'] - geom.box_protrusion),
        abs(right['perp_mean'] - geom.box_protrusion),
    )
    return gap_error, width_error, protrusion_error


def validate_boxes(
        clusters, tangential: np.ndarray, perp: np.ndarray, wall: WallLine,
        geom: DockGeometryParams, params: BoxValidationParams) -> CoarseDetection | None:
    if len(clusters) < 2:
        return None

    summaries = sorted(
        (_cluster_summary(tangential, perp, idx) for idx in clusters),
        key=lambda c: c['t_mid'],
    )

    best = None
    for left, right in zip(summaries[:-1], summaries[1:]):
        gap_error, width_error, protrusion_error = _pair_errors(left, right, geom)
        within_tolerance = (
            gap_error <= params.gap_tolerance
            and width_error <= params.width_tolerance
            and protrusion_error <= params.protrusion_tolerance
        )
        if not within_tolerance:
            continue

        score = (
            gap_error / params.gap_tolerance
            + width_error / params.width_tolerance
            + protrusion_error / params.protrusion_tolerance
        )
        if best is None or score < best[0]:
            t_mid_axis = (left['t_max'] + right['t_min']) / 2.0
            t_axis = rotate90(wall.normal)
            position = wall.point + t_mid_axis * t_axis
            yaw = float(np.arctan2(wall.normal[1], wall.normal[0]))
            pose = Pose2D(x=float(position[0]), y=float(position[1]), yaw=yaw)
            detection = CoarseDetection(
                pose=pose, gap_error=gap_error,
                width_error=width_error, protrusion_error=protrusion_error,
            )
            best = (score, detection)

    return best[1] if best else None


def coarse_confidence(
        coarse: CoarseDetection, wall: WallLine, ransac_p: RansacParams,
        params: BoxValidationParams) -> float:
    inlier_ratio_score = wall.inlier_ratio / max(ransac_p.min_inlier_ratio, 1e-6)
    inlier_score = float(np.clip(inlier_ratio_score, 0.0, 1.0))
    gap_score = max(0.0, 1.0 - coarse.gap_error / params.gap_tolerance)
    width_score = max(0.0, 1.0 - coarse.width_error / params.width_tolerance)
    protrusion_score = max(0.0, 1.0 - coarse.protrusion_error / params.protrusion_tolerance)
    total = (0.4 * inlier_score + 0.2 * gap_score
             + 0.2 * width_score + 0.2 * protrusion_score)
    return float(np.clip(total, 0.0, 1.0))
