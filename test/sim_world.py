"""
Simulador uniciclo mínimo del escenario del reto, para tests y benchmark del controller.

Solo es un banco de pruebas (no forma parte del paquete entregado): la pose del
marcador se le entrega al controller como si viniera del detector, con ruido
opcional. La geometría (pared en x = 1.95, cajas 8x8 cm con hueco de 9.5 cm)
imita `dock_challenge.world`; `is_docked` aproxima las condiciones del README
(centro a < 26.8 cm de la pared, ~8 mm lateral, ±6° de yaw).
"""
from dataclasses import dataclass, field
import math

from create3_dock_solution.control.dock_targets import dock_goal_from_marker
from create3_dock_solution.control.params import ControllerParams
from create3_dock_solution.controller import DockController, DOCKED, FAILED
from create3_dock_solution.perception.geometry import integrate_unicycle, pose_relative
from create3_dock_solution.types import DockEstimate, HazardState, Pose2D
import numpy as np

ROBOT_RADIUS = 0.1695
BOX_SIZE = 0.08
BOX_GAP = 0.095


@dataclass
class SimConfig:
    marker: Pose2D = field(default_factory=lambda: Pose2D(1.95, 0.0, math.pi))
    true_dock_offset: float = 0.266
    docked_wall_dist: float = 0.268
    docked_lateral: float = 0.008
    docked_yaw_deg: float = 6.0
    contact_wall_dist: float = 0.258    # el frente toca el dock: no se puede avanzar más
    noise_pos: float = 0.0              # sigma de la estimación del marcador [m]
    noise_yaw_deg: float = 0.0
    detect_range: float = 3.0
    dt: float = 0.05
    scan_period_ticks: int = 2          # 10 Hz de detección con control a 20 Hz
    t_max: float = 180.0
    seed: int = 0


@dataclass
class EpisodeResult:
    docked: bool
    state: str
    t_docked: float | None
    t_predock: float | None
    e_lat: float
    e_yaw_deg: float
    min_clearance: float
    attempts: int
    controller: DockController
    trajectory: list


def _box_clearance(robot: Pose2D, marker: Pose2D) -> float:
    """Distancia del borde del robot a la caja más cercana (negativa = choque)."""
    rel = pose_relative(marker, robot)   # x: hacia la sala, y: a lo largo de la pared
    best = math.inf
    for sign in (1.0, -1.0):
        y_lo = BOX_GAP / 2.0 if sign > 0 else -(BOX_GAP / 2.0 + BOX_SIZE)
        y_hi = y_lo + BOX_SIZE
        dx = max(0.0 - rel.x, 0.0, rel.x - BOX_SIZE)
        dy = max(y_lo - rel.y, 0.0, rel.y - y_hi)
        best = min(best, math.hypot(dx, dy) - ROBOT_RADIUS)
    return best


def run_episode(start: Pose2D, params: ControllerParams | None = None,
                cfg: SimConfig | None = None, recorder=None) -> EpisodeResult:
    cfg = cfg or SimConfig()
    params = params or ControllerParams()
    rng = np.random.default_rng(cfg.seed)
    ctrl = DockController(params)
    true_goal = dock_goal_from_marker(cfg.marker, cfg.true_dock_offset)
    robot = Pose2D(start.x, start.y, start.yaw)
    dock: DockEstimate | None = None
    min_clearance = math.inf
    t_docked = None
    trajectory = []
    if recorder is not None:
        recorder.start(0.0, params.approach.mode, params.to_nested_dict())

    n_steps = int(cfg.t_max / cfg.dt)
    for k in range(n_steps):
        now = k * cfg.dt
        if k % cfg.scan_period_ticks == 0:
            if math.hypot(cfg.marker.x - robot.x, cfg.marker.y - robot.y) <= cfg.detect_range:
                noisy = Pose2D(cfg.marker.x + rng.normal(0.0, cfg.noise_pos),
                               cfg.marker.y + rng.normal(0.0, cfg.noise_pos),
                               cfg.marker.yaw + math.radians(rng.normal(0.0, cfg.noise_yaw_deg)))
                dock = DockEstimate(noisy, 0.9, now)
            else:
                dock = None

        rel = pose_relative(true_goal, robot)
        wall_dist = cfg.true_dock_offset - rel.x
        yaw_err = math.degrees(abs(rel.yaw))
        is_docked = (wall_dist < cfg.docked_wall_dist and abs(rel.y) < cfg.docked_lateral
                     and yaw_err < cfg.docked_yaw_deg)
        clearance = _box_clearance(robot, cfg.marker)
        min_clearance = min(min_clearance, clearance)
        hazards = HazardState(bump=clearance < 0.0 or wall_dist <= cfg.contact_wall_dist)

        cmd = ctrl.step(dock, robot, is_docked, hazards, now)
        if recorder is not None:
            recorder.record(now, robot, ctrl.last_debug, hazards)
        trajectory.append((now, robot.x, robot.y, robot.yaw, ctrl.state))
        if ctrl.state == DOCKED and t_docked is None:
            t_docked = now
        if ctrl.state in (DOCKED, FAILED):
            break

        new_robot = integrate_unicycle(robot, cmd.v, cmd.w, cfg.dt)
        # Contacto frontal con el dock: no avanza más allá (sí puede girar/retroceder).
        if cfg.true_dock_offset - pose_relative(true_goal, new_robot).x < cfg.contact_wall_dist \
                and cmd.v > 0.0:
            new_robot = Pose2D(robot.x, robot.y, new_robot.yaw)
        robot = new_robot

    rel = pose_relative(true_goal, robot)
    t_predock = next((t for t, kind, _ in ctrl.events if kind == 'predock'), None)
    if recorder is not None:
        recorder.finish('docked' if ctrl.state == DOCKED else ctrl.state.lower(),
                        ctrl.events, ctrl.t_start, ctrl.attempts)
    return EpisodeResult(
        docked=ctrl.state == DOCKED, state=ctrl.state, t_docked=t_docked, t_predock=t_predock,
        e_lat=rel.y, e_yaw_deg=math.degrees(rel.yaw), min_clearance=min_clearance,
        attempts=ctrl.attempts, controller=ctrl, trajectory=trajectory)


def evaluation_grid(nx: int = 4, ny: int = 5, nyaw: int = 6) -> list[Pose2D]:
    """Poses del rango de evaluación del reto: x∈[0,1.3], y∈[-0.9,0.9], yaw∈[-π,π)."""
    poses = []
    for x in np.linspace(0.0, 1.3, nx):
        for y in np.linspace(-0.9, 0.9, ny):
            for yaw in np.linspace(-math.pi, math.pi, nyaw, endpoint=False):
                poses.append(Pose2D(float(x), float(y), float(yaw)))
    return poses
