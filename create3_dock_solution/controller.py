"""
Máquina de estados del docking (Python puro, sin rclpy).

    SEARCH -> GO_TO_PREDOCK -> [ALIGN_AT_PREDOCK] -> FINAL_APPROACH -> DOCKED
                   ^                  |                    |
                   |                  v (abort)            v (abort / bump / stall)
                   +-------------- BACK_OFF <--------------+
                                                   FAILED (tras max_attempts)

Basada en el controller del compañero (rama feat/robust-controller): ir al
pre-dock, alinear y aproximación final con corrección lateral continua,
abortos, reintentos, timeouts y watchdog de progreso. Cambios: SEARCH,
BACK_OFF, modo `polar` (B3) que pasa directo a FINAL, objetivo que se sigue
actualizando con el detector (se congela solo en los últimos cm), fin por
`is_docked` y reacción a /hazard_detection. Ver docs/CONTROLADOR_IMPLEMENTADO.md.
"""
import math
from typing import Callable

from .control.dock_targets import axis_errors, AxisErrors, dock_goal_from_marker, predock_from_goal
from .control.laws import axis_tracking, clamp, go_to_point, polar_to_pose, rate_limit, rotate_to
from .control.params import ControllerParams
from .perception.geometry import pose_relative, wrap_angle
from .types import Command, DockEstimate, HazardState, Pose2D

SEARCH = 'SEARCH'
GO_TO_PREDOCK = 'GO_TO_PREDOCK'
ALIGN_AT_PREDOCK = 'ALIGN_AT_PREDOCK'
FINAL_APPROACH = 'FINAL_APPROACH'
BACK_OFF = 'BACK_OFF'
DOCKED = 'DOCKED'
FAILED = 'FAILED'


class DockController:
    """
    Contrato con `dock_node`.

    `step(dock, robot_pose, is_docked, hazards, now) -> Command`, llamado a
    20 Hz. `dock` es la última `DockEstimate` del detector (pose del marcador en
    `odom`) o `None` si no hay detección fresca; `now` en segundos (mismo reloj
    que el nodo). Expone `state`, `last_debug` (objetivos y errores, para
    markers y el reporte) y `events` [(t desde el inicio, tipo, detalle)].
    """

    def __init__(self, params: ControllerParams | dict | None = None,
                 log: Callable[[str], None] | None = None):
        if not isinstance(params, ControllerParams):
            params = ControllerParams.from_dict(params or {})
        self.p = params
        self._log = log or (lambda _msg: None)
        self.reset()

    def set_params(self, params: ControllerParams):
        """Cambia parámetros en caliente (p.ej. `ros2 param set`); el estado se conserva."""
        self.p = params

    def reset(self):
        self.state = SEARCH
        self.attempts = 0
        self.marker: Pose2D | None = None
        self.goal: Pose2D | None = None
        self.predock: Pose2D | None = None
        self.events: list[tuple[float, str, str]] = []
        self.last_debug: dict = {}
        self.t_start: float | None = None
        self._frozen = False
        self._now = 0.0
        self._robot: Pose2D | None = None
        self._state_start = 0.0
        self._prev_cmd = Command(0.0, 0.0)
        self._search_phase = 'wait'
        self._search_phase_start = 0.0
        self._search_prev_yaw: float | None = None
        self._search_turned = 0.0
        self._backoff_start: Pose2D | None = None
        self._best_e_long = -math.inf
        self._best_e_long_time = 0.0
        self._undocked_since: float | None = None
        self._hold_e_long: float | None = None

    # ------------------------------------------------------------------
    def step(self, dock: DockEstimate | None, robot_pose: Pose2D, is_docked: bool,
             hazards: HazardState | None, now: float) -> Command:
        if self.t_start is None:
            self.t_start = now
            self._state_start = now
            self._search_phase_start = now
            self._now = now
            self._event('state', SEARCH)
        dt = max(0.0, now - self._now)
        self._now = now
        self._robot = robot_pose
        hazards = hazards or HazardState()

        self._update_target(dock, robot_pose)
        errors = axis_errors(self.goal, robot_pose) if self.goal is not None else None

        if is_docked and self.state != DOCKED:
            self._set_state(DOCKED, f'is_docked en {now - self.t_start:.1f} s')

        emergency = False
        v, w = 0.0, 0.0
        hazard_action = self._handle_hazards(hazards, errors)
        if hazard_action == 'stop':
            emergency = True
        else:
            if hazard_action == 'backoff':
                emergency = True
            self._check_timeouts()
            v, w = self._dispatch(dock, robot_pose, is_docked, hazards, errors)

        cmd = self._shape_output(v, w, dt, emergency)
        self._update_debug(errors, cmd, dock)
        return cmd

    # ------------------------------------------------------------------
    # Objetivo
    # ------------------------------------------------------------------
    def _update_target(self, dock: DockEstimate | None, robot: Pose2D):
        if self.state == FINAL_APPROACH and self.goal is not None and not self._frozen:
            if axis_errors(self.goal, robot).e_long > -self.p.final.freeze_within_m:
                self._frozen = True
                self._event('freeze', 'objetivo congelado')
        if dock is not None and not self._frozen:
            if self.marker is None:
                self._event('first_detection', f'confidence={dock.confidence:.2f}')
            self.marker = dock.pose
        if self.marker is not None and not self._frozen:
            g = self.p.geometry
            self.goal = dock_goal_from_marker(self.marker, g.dock_offset_m)
            self.predock = predock_from_goal(self.goal, g.predock_distance_m)

    # ------------------------------------------------------------------
    # Hazards
    # ------------------------------------------------------------------
    def _handle_hazards(self, hz: HazardState, errors: AxisErrors | None) -> str | None:
        if self.state in (DOCKED, FAILED):
            return None
        if hz.cliff:
            self._log_throttled('CLIFF/WHEEL_DROP: detenido')
            return 'stop'
        if not (hz.bump or hz.stall) or self.state == BACK_OFF:
            return None
        near_dock = (self.state == FINAL_APPROACH and errors is not None
                     and errors.e_long > -self.p.hazard.bump_ignore_within_m)
        if near_dock:
            return None     # contacto esperado con el dock: seguir hasta is_docked
        kind = 'bump' if hz.bump else 'stall'
        self._event('hazard', kind)
        counts = self.state in (GO_TO_PREDOCK, ALIGN_AT_PREDOCK, FINAL_APPROACH)
        self._abort(f'{kind} en {self.state}', back_off=True, count=counts)
        return 'backoff'

    # ------------------------------------------------------------------
    # Transiciones
    # ------------------------------------------------------------------
    def _check_timeouts(self):
        s = self.p.supervision
        limits = {GO_TO_PREDOCK: s.approach_timeout_s, ALIGN_AT_PREDOCK: s.align_timeout_s,
                  FINAL_APPROACH: s.final_timeout_s}
        limit = limits.get(self.state)
        if limit is not None and self._now - self._state_start > limit:
            self._abort(f'timeout en {self.state} (> {limit:.0f} s)',
                        back_off=self.state == FINAL_APPROACH)

    def _abort(self, reason: str, back_off: bool, count: bool = True):
        if count:
            self.attempts += 1
            if self.attempts >= self.p.supervision.max_attempts:
                self._set_state(FAILED, f'{reason} (intento {self.attempts})')
                return
        target = BACK_OFF if back_off else (GO_TO_PREDOCK if self.goal is not None else SEARCH)
        self._set_state(target, f'{reason} (intentos {self.attempts}/'
                                f'{self.p.supervision.max_attempts})')

    def _set_state(self, new_state: str, reason: str = ''):
        old = self.state
        if old == new_state:
            return
        if old == FINAL_APPROACH and new_state != DOCKED:
            self._frozen = False
        self.state = new_state
        self._state_start = self._now
        if new_state == SEARCH:
            self._search_phase = 'wait'
            self._search_phase_start = self._now
            self._search_prev_yaw = None
            self._search_turned = 0.0
        elif new_state == FINAL_APPROACH:
            self._best_e_long = -math.inf
            self._best_e_long_time = self._now
        elif new_state == BACK_OFF:
            self._backoff_start = self._robot
        elif new_state == DOCKED:
            self._undocked_since = None
            self._hold_e_long = None
        self._event('state', new_state, reason)
        t = self._now - (self.t_start or self._now)
        self._log(f'[{t:6.2f} s] {old} -> {new_state}' + (f' ({reason})' if reason else ''))

    def _event(self, kind: str, detail: str = '', extra: str = ''):
        t = self._now - (self.t_start if self.t_start is not None else self._now)
        self.events.append((t, kind, f'{detail} {extra}'.strip()))

    def _log_throttled(self, msg: str):
        if not self.events or self.events[-1][2] != msg:
            self._event('info', msg)
            self._log(msg)

    # ------------------------------------------------------------------
    # Estados
    # ------------------------------------------------------------------
    def _dispatch(self, dock, robot, is_docked, hazards, errors) -> tuple[float, float]:
        # Un estado puede transicionar y ceder el tick al siguiente (sin perder 50 ms).
        for _ in range(3):
            state = self.state
            handler = {
                SEARCH: self._search, GO_TO_PREDOCK: self._approach,
                ALIGN_AT_PREDOCK: self._align, FINAL_APPROACH: self._final,
                BACK_OFF: self._back_off, DOCKED: self._docked, FAILED: self._failed,
            }[state]
            result = handler(dock, robot, is_docked, hazards, errors)
            if result is not None:
                return result
            errors = axis_errors(self.goal, robot) if self.goal is not None else None
        return 0.0, 0.0

    def _search(self, dock, robot, is_docked, hazards, errors):
        if self.goal is not None and (
                dock is None or dock.confidence >= self.p.supervision.min_confidence):
            self._set_state(GO_TO_PREDOCK, 'dock detectado')
            return None
        sp = self.p.search
        t_phase = self._now - self._search_phase_start
        if self._search_phase == 'wait':
            if t_phase < sp.wait_s:
                return 0.0, 0.0
            self._search_phase, self._search_phase_start = 'rotate', self._now
            self._search_prev_yaw, self._search_turned = robot.yaw, 0.0
        if self._search_phase == 'rotate':
            self._search_turned += abs(wrap_angle(robot.yaw - self._search_prev_yaw))
            self._search_prev_yaw = robot.yaw
            if self._search_turned < 2.0 * math.pi:
                return 0.0, sp.w
            self._search_phase, self._search_phase_start = 'arc', self._now
            self._event('search', 'vuelta completa sin detección: arco')
            t_phase = 0.0
        if t_phase < sp.arc_time_s:
            scale = self.p.hazard.proximity_speed_scale if hazards.proximity else 1.0
            return sp.arc_v * scale, sp.arc_w
        self._search_phase, self._search_phase_start = 'rotate', self._now
        self._search_prev_yaw, self._search_turned = robot.yaw, 0.0
        return 0.0, sp.w

    def _approach(self, dock, robot, is_docked, hazards, errors):
        if self.goal is None:
            self._set_state(SEARCH, 'sin objetivo')
            return None
        ap = self.p.approach
        rel = pose_relative(self.predock, robot)
        rho = math.hypot(rel.x, rel.y)
        if ap.mode == 'polar':
            pp = self.p.polar
            if rho < pp.switch_dist_m or (rho < 2.0 * pp.switch_dist_m and rel.x > 0.0):
                aligned = (abs(errors.e_yaw) < math.radians(pp.switch_yaw_deg)
                           and abs(errors.e_lat) < pp.switch_lat_m)
                self._event('predock', 'polar')
                if aligned:
                    self._set_state(FINAL_APPROACH, 'pre-dock alineado (polar)')
                else:
                    self._set_state(ALIGN_AT_PREDOCK, 'pre-dock desalineado (polar)')
                return None
            v, w = polar_to_pose(robot, self.predock, ap, pp)
        else:
            heading_err = wrap_angle(
                math.atan2(self.predock.y - robot.y, self.predock.x - robot.x) - robot.yaw)
            passed = rho < 1.5 * ap.tolerance_m and abs(heading_err) > math.pi / 2.0
            if rho < ap.tolerance_m or passed:
                self._event('predock', 'point_align')
                self._set_state(ALIGN_AT_PREDOCK, f'pre-dock a {rho * 100:.1f} cm')
                return None
            v, w = go_to_point(robot, self.predock, ap)
        if hazards.proximity:
            v *= self.p.hazard.proximity_speed_scale
        return v, w

    def _align(self, dock, robot, is_docked, hazards, errors):
        al = self.p.align
        if abs(errors.e_lat) > al.abort_lat_m:
            self._abort(f'fuera del eje en el pre-dock (e_lat={errors.e_lat:.3f} m)',
                        back_off=False)
            return None
        if abs(errors.e_yaw) < math.radians(al.tolerance_deg):
            self._set_state(FINAL_APPROACH, 'alineado')
            return None
        return 0.0, rotate_to(errors.e_yaw, al)

    def _final(self, dock, robot, is_docked, hazards, errors):
        fp = self.p.final
        e = errors
        if e.e_long > fp.overshoot_m:
            self._abort(f'se pasó del dock sin is_docked (e_long={e.e_long:.3f} m)', back_off=True)
            return None
        if abs(e.e_lat) > fp.abort_lat_m or abs(e.e_yaw) > math.radians(fp.abort_yaw_deg):
            self._abort(f'desvío en FINAL (e_lat={e.e_lat:.3f} m, '
                        f'e_yaw={math.degrees(e.e_yaw):.1f} deg)', back_off=True)
            return None
        s = self.p.supervision
        if e.e_long > self._best_e_long + s.progress_eps_m:
            self._best_e_long, self._best_e_long_time = e.e_long, self._now
        elif self._now - self._best_e_long_time > s.progress_timeout_s:
            self._abort(f'sin progreso en FINAL por {s.progress_timeout_s:.0f} s', back_off=True)
            return None
        return axis_tracking(e.e_long, e.e_lat, e.e_yaw, fp)

    def _back_off(self, dock, robot, is_docked, hazards, errors):
        bp = self.p.backoff
        start = self._backoff_start or robot
        travelled = math.hypot(robot.x - start.x, robot.y - start.y)
        if travelled >= bp.distance_m or self._now - self._state_start > bp.timeout_s:
            next_state = GO_TO_PREDOCK if self.goal is not None else SEARCH
            self._set_state(next_state, f'retrocedió {travelled:.2f} m')
            return None
        return -bp.v, 0.0

    def _docked(self, dock, robot, is_docked, hazards, errors):
        if is_docked:
            self._undocked_since = None
        elif self._undocked_since is None:
            self._undocked_since = self._now
        elif self._now - self._undocked_since > self.p.supervision.undock_grace_s:
            self._set_state(FINAL_APPROACH, 'is_docked se perdió')
            return None
        s = self.p.supervision
        if errors is None or s.docked_hold_v <= 0.0:
            return 0.0, 0.0
        if self._hold_e_long is None:
            self._hold_e_long = errors.e_long + s.docked_hold_advance_m
        v = clamp(s.docked_hold_k * (self._hold_e_long - errors.e_long),
                  -s.docked_hold_v, s.docked_hold_v)
        return v, 0.0

    def _failed(self, dock, robot, is_docked, hazards, errors):
        return 0.0, 0.0

    # ------------------------------------------------------------------
    def _shape_output(self, v: float, w: float, dt: float, emergency: bool) -> Command:
        lim = self.p.limits
        v = clamp(v, -lim.v_abs_max, lim.v_abs_max)
        w = clamp(w, -lim.w_abs_max, lim.w_abs_max)
        if emergency:
            # Parada inmediata (sin rampa); la rampa sigue desde 0.
            self._prev_cmd = Command(0.0, 0.0)
            return Command(0.0, 0.0)
        if self.state == FAILED:
            v, w = 0.0, 0.0
        else:
            v = rate_limit(v, self._prev_cmd.v, lim.max_lin_acc, dt)
            w = rate_limit(w, self._prev_cmd.w, lim.max_ang_acc, dt)
        self._prev_cmd = Command(v, w)
        return Command(v, w)

    def _update_debug(self, errors: AxisErrors | None, cmd: Command, dock):
        dist_predock = None
        if self.predock is not None and self._robot is not None:
            dist_predock = math.hypot(self.predock.x - self._robot.x,
                                      self.predock.y - self._robot.y)
        self.last_debug = {
            'state': self.state, 'mode': self.p.approach.mode, 'attempts': self.attempts,
            'marker': self.marker, 'goal': self.goal, 'predock': self.predock,
            'frozen': self._frozen,
            'e_long': errors.e_long if errors else None,
            'e_lat': errors.e_lat if errors else None,
            'e_yaw': errors.e_yaw if errors else None,
            'dist_predock': dist_predock,
            'confidence': dock.confidence if dock is not None else None,
            'v': cmd.v, 'w': cmd.w,
        }
