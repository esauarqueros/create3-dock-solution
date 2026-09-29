import numpy as np

from .perception.box_validator import (
    cluster_points, coarse_confidence, extract_protrusion_points, validate_boxes,
)
from .perception.dock_model import (
    BoxValidationParams, build_template, DockGeometryParams, FilterParams, IcpParams, RansacParams,
)
from .perception.geometry import polar_to_points, pose_compose, transform_points
from .perception.icp_refiner import icp_confidence, is_refinement_plausible
from .perception.icp_refiner import refine as icp_refine
from .perception.temporal_filter import DockPoseFilter
from .perception.wall_ransac import fit_wall
from .types import DockEstimate, Pose2D


class DockDetector:
    """
    Detección del dock a partir de un scan LIDAR ya expresado en `base_link`.

    RANSAC de pared + validación geométrica de las cajas (A2+A1) a cualquier
    distancia, refinado con ICP (A3) cuando el robot está lo bastante cerca.
    La pose final se filtra en `odom` (EMA adaptativa por confidence).

    Devuelve `None` cuando no hay una detección confiable -- p.ej. el caso en que
    el robot arranca a un costado del dock y no puede ver ambas cajas -- para que
    `controller.py` pueda construir su propia lógica de recuperación sobre esa señal.
    """

    def __init__(self, params: dict):
        params = params or {}
        self.geom = DockGeometryParams.from_dict(params.get('dock_geometry', {}))
        self.ransac_p = RansacParams.from_dict(params.get('ransac', {}))
        self.box_p = BoxValidationParams.from_dict(params.get('box_validation', {}))
        self.icp_p = IcpParams.from_dict(params.get('icp', {}))
        self.filter = DockPoseFilter(FilterParams.from_dict(params.get('filter', {})))
        self.template = build_template(self.geom)
        self._rng = np.random.default_rng(params.get('random_seed', 42))
        self.last_debug: dict = {}

    def update(self, ranges: np.ndarray, angle_min: float, angle_inc: float,
               robot_pose: Pose2D, stamp: float, laser_to_base: Pose2D | None = None,
               range_min: float = 0.05, range_max: float = 25.0) -> DockEstimate | None:
        self.last_debug = {'wall': None, 'coarse_pose': None, 'refined_pose': None}

        pts_laser = polar_to_points(ranges, angle_min, angle_inc, range_min, range_max)
        if pts_laser.shape[0] == 0:
            return self.filter.update(None, 0.0, stamp)
        if laser_to_base is not None:
            pts_base = transform_points(pts_laser, laser_to_base)
        else:
            pts_base = pts_laser

        # Restringir al rango donde se espera el dock: evita que la mayoría de
        # puntos del cuarto (paredes lejanas, lecturas a rango máximo) diluyan la
        # proporción de inliers de la pared del dock durante el RANSAC.
        near_mask = np.hypot(pts_base[:, 0], pts_base[:, 1]) <= self.geom.wall_search_range_max
        pts_near = pts_base[near_mask]

        wall = fit_wall(pts_near, self.ransac_p, self._rng)
        if wall is None:
            return self.filter.update(None, 0.0, stamp)
        self.last_debug['wall'] = wall

        tangential, perp = extract_protrusion_points(pts_near, wall, self.box_p)
        clusters = cluster_points(tangential, self.box_p)

        coarse = validate_boxes(clusters, tangential, perp, wall, self.geom, self.box_p)
        if coarse is None:
            return self.filter.update(None, 0.0, stamp)
        self.last_debug['coarse_pose'] = coarse.pose

        final_pose = coarse.pose
        confidence = coarse_confidence(coarse, wall, self.ransac_p, self.box_p)

        distance = float(np.hypot(coarse.pose.x, coarse.pose.y))
        if self.icp_p.enable and distance <= self.icp_p.coarse_to_fine_range_m:
            refined = icp_refine(pts_near, self.template, coarse.pose, self.icp_p)
            plausible = refined is not None and is_refinement_plausible(
                coarse.pose, refined.pose, self.icp_p)
            if plausible:
                final_pose = refined.pose
                confidence = icp_confidence(refined, self.icp_p)
                self.last_debug['refined_pose'] = refined.pose

        pose_odom = pose_compose(robot_pose, final_pose)
        return self.filter.update(pose_odom, confidence, stamp)
