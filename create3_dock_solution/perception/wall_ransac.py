"""
A2 -- RANSAC de pared: ajusta una recta a los puntos del scan y da su normal.

La normal se orienta siempre hacia el origen del frame en que se llama (es decir,
hacia el robot), lo cual asume que `points` ya están en `base_link` (robot en el
origen) -- responsabilidad de quien llama (`DockDetector`).
"""
from dataclasses import dataclass

import numpy as np

from .dock_model import RansacParams


@dataclass
class WallLine:
    point: np.ndarray       # un punto sobre la recta (centroide de inliers), en base_link
    normal: np.ndarray      # normal unitaria, apunta hacia el robot (origen)
    inlier_ratio: float
    num_inliers: int
    inlier_mask: np.ndarray | None = None   # máscara sobre los puntos pasados a fit_wall


def fit_wall(points: np.ndarray, params: RansacParams,
             rng: np.random.Generator) -> WallLine | None:
    n = points.shape[0]
    if n < params.min_inliers:
        return None

    best_count = 0
    best_mask = None
    for _ in range(params.max_iterations):
        i, j = rng.choice(n, size=2, replace=False)
        p1, p2 = points[i], points[j]
        d = p2 - p1
        length = np.linalg.norm(d)
        if length < 1e-6:
            continue
        normal_candidate = np.array([-d[1], d[0]]) / length
        dist = np.abs((points - p1) @ normal_candidate)
        mask = dist < params.inlier_threshold
        count = int(mask.sum())
        if count > best_count:
            best_count, best_mask = count, mask

    if best_mask is None:
        return None
    ratio = best_count / n
    if best_count < params.min_inliers or ratio < params.min_inlier_ratio:
        return None

    inliers = points[best_mask]
    centroid = inliers.mean(axis=0)
    _, _, vt = np.linalg.svd(inliers - centroid)
    direction = vt[0]
    normal = np.array([-direction[1], direction[0]])
    normal = normal / np.linalg.norm(normal)
    if np.dot(normal, -centroid) < 0.0:
        normal = -normal

    return WallLine(
        point=centroid, normal=normal, inlier_ratio=ratio,
        num_inliers=best_count, inlier_mask=best_mask)
