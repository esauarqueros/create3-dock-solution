from .types import DockEstimate, Pose2D, Command

class DockController:
    def __init__(self, params: dict):
        self.p = params
        self.state = 'SEARCH'   # SEARCH, APPROACH, ALIGN, FINAL, DOCKED

    def step(self, dock: DockEstimate | None, robot_pose: Pose2D,
             is_docked: bool, dt: float) -> Command:
        raise NotImplementedError