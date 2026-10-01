import math

from create3_dock_solution.control.params import ControllerParams
from create3_dock_solution.controller import (
    BACK_OFF, DockController, DOCKED, GO_TO_PREDOCK, SEARCH,
)
from create3_dock_solution.types import DockEstimate, HazardState, Pose2D
import pytest
from sim_world import run_episode, SimConfig

MARKER = Pose2D(1.95, 0.0, math.pi)
START_POSES = [
    Pose2D(0.41, -0.18, math.radians(20.6)),    # pose por defecto del reto
    Pose2D(0.0, 0.0, math.pi),                  # de espaldas
    Pose2D(1.3, 0.9, -math.pi / 2),             # a un costado, cerca de la pared
    Pose2D(1.3, -0.9, math.pi / 2),
    Pose2D(0.0, 0.9, 0.0),
]


def _params(mode: str) -> ControllerParams:
    return ControllerParams.from_dict({'approach': {'mode': mode}})


@pytest.mark.parametrize('mode', ['point_align', 'polar'])
@pytest.mark.parametrize('start', START_POSES)
def test_controller_docks_from_evaluation_poses(mode, start):
    result = run_episode(start, _params(mode), SimConfig(noise_pos=0.002, noise_yaw_deg=0.2))
    assert result.docked, f'terminó en {result.state}'
    assert result.t_docked < 45.0
    assert abs(result.e_lat) < 0.008
    assert abs(result.e_yaw_deg) < 6.0
    assert result.min_clearance > 0.0


def test_commands_respect_limits():
    params = ControllerParams()
    result = run_episode(Pose2D(0.0, 0.0, math.pi), params)
    assert result.docked
    ctrl = result.controller
    assert abs(ctrl.last_debug['v']) <= params.limits.v_abs_max


def test_search_goes_straight_to_approach_when_dock_already_detected():
    ctrl = DockController(ControllerParams())
    dock = DockEstimate(MARKER, 0.9, 0.0)
    ctrl.step(dock, Pose2D(0.4, 0.0, 0.0), False, HazardState(), 0.0)
    assert ctrl.state == GO_TO_PREDOCK


def test_search_rotates_in_place_without_detection():
    params = ControllerParams()
    ctrl = DockController(params)
    robot = Pose2D(0.4, 0.0, 0.0)
    cmd = None
    for k in range(40):
        cmd = ctrl.step(None, robot, False, HazardState(), k * 0.05)
    assert ctrl.state == SEARCH
    assert cmd.v == 0.0
    assert cmd.w > 0.0


def test_bump_during_approach_triggers_back_off():
    ctrl = DockController(ControllerParams())
    dock = DockEstimate(MARKER, 0.9, 0.0)
    robot = Pose2D(0.4, 0.0, 0.0)
    for k in range(10):
        ctrl.step(dock, robot, False, HazardState(), k * 0.05)
    cmd = ctrl.step(dock, robot, False, HazardState(bump=True), 0.5)
    assert ctrl.state == BACK_OFF
    assert cmd.v == 0.0                     # parada inmediata, luego retrocede
    cmd = ctrl.step(dock, robot, False, HazardState(), 0.55)
    assert cmd.v < 0.0


def test_target_is_held_when_detection_is_lost():
    ctrl = DockController(ControllerParams())
    robot = Pose2D(0.4, 0.0, 0.0)
    ctrl.step(DockEstimate(MARKER, 0.9, 0.0), robot, False, HazardState(), 0.0)
    goal = ctrl.goal
    cmd = ctrl.step(None, robot, False, HazardState(), 0.05)
    assert ctrl.state == GO_TO_PREDOCK
    assert ctrl.goal == goal
    assert cmd.v >= 0.0


def test_is_docked_stops_the_robot():
    ctrl = DockController(ControllerParams())
    cmd = ctrl.step(DockEstimate(MARKER, 0.9, 0.0), Pose2D(1.68, 0.0, 0.0), True,
                    HazardState(), 0.0)
    assert ctrl.state == DOCKED
    assert cmd.w == 0.0
    assert abs(cmd.v) <= ControllerParams().supervision.docked_hold_v


def test_invalid_params_are_reported():
    params = ControllerParams.from_dict({'approach': {'mode': 'nope'}, 'polar': {'k_beta': 0.5}})
    assert len(params.validate()) == 2
    assert ControllerParams().validate() == []
