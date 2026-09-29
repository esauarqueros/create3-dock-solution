import numpy as np
from .types import DockEstimate, Pose2D

class DockDetector:
    def __init__(self, params: dict):
        self.p = params
        self._filtered = None   # el filtro en odom vive aquí

    def update(self, ranges: np.ndarray, angle_min: float, angle_inc: float,
               robot_pose: Pose2D, stamp: float) -> DockEstimate | None:
        """RANSAC pared + cajas → pose en frame láser → transforma a odom → filtra."""
        raise NotImplementedError