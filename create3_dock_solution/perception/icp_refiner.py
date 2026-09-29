"""
A3 -- refinamiento por ICP 2D punto-a-punto (cKDTree + Kabsch/SVD).

Sin dependencias externas más allá de numpy/scipy. Se usa a corta distancia,
cuando caen suficientes puntos sobre las cajas como para que valga la pena.
"""
from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree

from .geometry import matrix_to_pose, pose_to_matrix, wrap_angle
from ..types import Pose2D


@dataclass
class IcpResult:
    pose: Pose2D
    rms: float
    num_correspondences: int


def refine(points_base: np.ndarray, template_local: np.ndarray, init_pose: Pose2D,
           params) -> IcpResult | None:
    """
    Refina `init_pose` alineando `template_local` contra `points_base`.

    Asume que `init_pose` ya es una estimación razonablemente cercana (la del
    detector coarse validado geométricamente por box_validator, típicamente a
    pocos cm/grados) -- esto es un refinamiento local, no una búsqueda global:
    con un desplazamiento inicial grande, la simetría del molde pared+2-cajas
    puede converger a un mínimo local incorrecto (ver `is_refinement_plausible`).
    """
    if template_local.shape[0] == 0 or points_base.shape[0] == 0:
        return None

    r, t = pose_to_matrix(init_pose)
    tree = cKDTree(points_base)

    for _ in range(params.max_iterations):
        transformed = template_local @ r.T + t
        distances, indices = tree.query(transformed)
        mask = distances < params.max_correspondence_dist
        if int(mask.sum()) < params.min_correspondences:
            return None

        src = transformed[mask]
        dst = points_base[indices[mask]]

        src_c = src.mean(axis=0)
        dst_c = dst.mean(axis=0)
        h = (src - src_c).T @ (dst - dst_c)
        u, _, vt = np.linalg.svd(h)
        d = np.sign(np.linalg.det(vt.T @ u.T)) or 1.0
        correction = np.diag([1.0, d])
        r_delta = vt.T @ correction @ u.T
        t_delta = dst_c - r_delta @ src_c

        r = r_delta @ r
        t = r_delta @ t + t_delta

        d_translation = float(np.linalg.norm(t_delta))
        d_rotation = abs(float(np.arctan2(r_delta[1, 0], r_delta[0, 0])))
        converged = (d_translation < params.convergence_translation_eps
                     and d_rotation < params.convergence_rotation_eps_rad)
        if converged:
            break

    transformed = template_local @ r.T + t
    distances, indices = tree.query(transformed)
    mask = distances < params.max_correspondence_dist
    n_corr = int(mask.sum())
    if n_corr < params.min_correspondences:
        return None
    rms = float(np.sqrt(np.mean(distances[mask] ** 2)))
    if rms > params.max_residual_rms:
        return None

    return IcpResult(pose=matrix_to_pose(r, t), rms=rms, num_correspondences=n_corr)


def is_refinement_plausible(coarse_pose: Pose2D, refined_pose: Pose2D, params) -> bool:
    """
    Chequeo de plausibilidad geométrica del refinamiento ICP.

    Rechaza refinamientos que hayan convergido a un mínimo local lejos de la
    estimación gruesa ya validada geométricamente (p.ej. otra esquina de la sala).
    """
    dx = refined_pose.x - coarse_pose.x
    dy = refined_pose.y - coarse_pose.y
    translation_delta = float(np.hypot(dx, dy))
    rotation_delta = abs(wrap_angle(refined_pose.yaw - coarse_pose.yaw))
    return (translation_delta <= params.max_translation_correction_m
            and rotation_delta <= params.max_rotation_correction_rad)


def icp_confidence(result: IcpResult, params) -> float:
    residual_score = max(0.0, 1.0 - result.rms / params.max_residual_rms)
    coverage_score = min(1.0, result.num_correspondences / max(params.min_correspondences * 2, 1))
    return float(np.clip(0.7 * residual_score + 0.3 * coverage_score, 0.0, 1.0))
