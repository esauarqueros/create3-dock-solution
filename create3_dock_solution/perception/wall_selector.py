"""
Selección de la pared del dock entre varias paredes candidatas visibles.

Solo la pared del dock tiene la firma de las dos cajas; las demás paredes de
la sala son líneas lisas. En vez de quedarse con la única línea de mayor
cantidad de inliers de un solo RANSAC (que puede ser cualquier pared cercana),
se prueban varias líneas candidatas (RANSAC iterativo, removiendo los inliers
de cada candidata ya probada) y se usa la validación de cajas (A1) como
criterio de selección de cuál es la pared correcta -- no solo como validador
de una única pared ya elegida. Ver docs/ESTRATEGIA.md.
"""
import numpy as np

from .box_validator import (
    adapt_box_params, cluster_points, coarse_confidence, CoarseDetection,
    extract_protrusion_points, validate_boxes,
)
from .dock_model import BoxValidationParams, DockGeometryParams, RansacParams
from .wall_ransac import fit_wall, WallLine


def find_dock_wall(
        points: np.ndarray, geom: DockGeometryParams, ransac_p: RansacParams,
        box_p: BoxValidationParams, rng: np.random.Generator, angle_inc: float,
        max_candidates: int) -> tuple[float, WallLine, CoarseDetection] | None:
    """
    Prueba hasta `max_candidates` paredes y devuelve la de mayor confidence.

    No se queda con la primera candidata que valide: evalúa todas las
    disponibles (hasta el límite) y elige la de mayor `coarse_confidence`,
    para que una pared lisa cualquiera no "gane" por ruido antes de llegar a
    la pared real del dock. Devuelve `None` si ninguna candidata valida.
    """
    remaining = points
    best: tuple[float, WallLine, CoarseDetection] | None = None

    for _ in range(max_candidates):
        wall = fit_wall(remaining, ransac_p, rng)
        if wall is None:
            break

        distance = abs(float(np.dot(wall.point, wall.normal)))
        adapted = adapt_box_params(box_p, distance, angle_inc)

        tangential, perp = extract_protrusion_points(remaining, wall, adapted)
        clusters = cluster_points(tangential, adapted)
        coarse = validate_boxes(clusters, tangential, perp, wall, geom, adapted)

        if coarse is not None:
            confidence = coarse_confidence(
                coarse, wall, ransac_p, adapted, geom, angle_inc, distance)
            if best is None or confidence > best[0]:
                best = (confidence, wall, coarse)

        remaining = remaining[~wall.inlier_mask]
        if remaining.shape[0] < ransac_p.min_inliers:
            break

    return best
