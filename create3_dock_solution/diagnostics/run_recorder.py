"""
Reporte por corrida: tiempos (arranque -> pre-dock -> dock), precisión estimada y gráfica.

Python puro (sin rclpy): lo usa `dock_node` (parámetro `debug.run_report.enable`)
y también el benchmark offline, así ambos producen el mismo formato:

    <output_dir>/<fecha>_<modo>[_<tag>]/timeseries.csv, summary.json, run.png

La precisión es la que estima nuestro propio detector (errores del robot
respecto al dock real calculado del marcador), no la del ground truth.
"""
import csv
from datetime import datetime
import json
import math
import os
from typing import Callable

from ..perception.geometry import pose_relative
from ..types import HazardState, Pose2D

TIME_LIMIT_S = 180.0
LATERAL_TOL_M = 0.008       # ±6° sobre el eje a la distancia de acople (README del reto)
YAW_TOL_DEG = 6.0

_COLUMNS = ['t', 'state', 'x', 'y', 'yaw', 'v', 'w', 'e_long', 'e_lat', 'e_yaw',
            'dist_predock', 'confidence', 'goal_x', 'goal_y', 'goal_yaw',
            'predock_x', 'predock_y', 'bump', 'stall', 'cliff', 'proximity']

_STATE_COLORS = {
    'SEARCH': '#9e9e9e', 'GO_TO_PREDOCK': '#42a5f5', 'ALIGN_AT_PREDOCK': '#ab47bc',
    'FINAL_APPROACH': '#ffa726', 'BACK_OFF': '#ef5350', 'DOCKED': '#66bb6a', 'FAILED': '#212121',
}


class RunRecorder:
    def __init__(self, output_dir: str, tag: str = '', save_plot: bool = True,
                 log: Callable[[str], None] | None = None):
        self.output_dir = os.path.expanduser(output_dir)
        self.tag = tag
        self.save_plot = save_plot
        self._log = log or print
        self.rows: list[dict] = []
        self.t0: float | None = None
        self.wall_start: str | None = None
        self.mode = ''
        self.params: dict = {}
        self.finished = False

    def start(self, t0: float, mode: str, params: dict):
        self.t0 = t0
        self.wall_start = datetime.now().isoformat(timespec='seconds')
        self.mode = mode
        self.params = params

    @property
    def started(self) -> bool:
        return self.t0 is not None

    def record(self, now: float, robot: Pose2D, debug: dict, hazards: HazardState | None):
        if not self.started or self.finished:
            return
        hazards = hazards or HazardState()
        goal, predock = debug.get('goal'), debug.get('predock')
        self.rows.append({
            't': now - self.t0, 'state': debug.get('state', ''),
            'x': robot.x, 'y': robot.y, 'yaw': robot.yaw,
            'v': debug.get('v'), 'w': debug.get('w'),
            'e_long': debug.get('e_long'), 'e_lat': debug.get('e_lat'),
            'e_yaw': debug.get('e_yaw'), 'dist_predock': debug.get('dist_predock'),
            'confidence': debug.get('confidence'),
            'goal_x': goal.x if goal else None, 'goal_y': goal.y if goal else None,
            'goal_yaw': goal.yaw if goal else None,
            'predock_x': predock.x if predock else None,
            'predock_y': predock.y if predock else None,
            'bump': int(hazards.bump), 'stall': int(hazards.stall),
            'cliff': int(hazards.cliff), 'proximity': int(hazards.proximity),
        })

    # ------------------------------------------------------------------
    def build_summary(self, status: str, events: list, events_t_start: float,
                      attempts: int = 0) -> dict:
        """`events` = DockController.events; sus tiempos son relativos a `events_t_start`."""
        offset = events_t_start - self.t0
        evs = [(t + offset, kind, detail) for t, kind, detail in events]

        def first(kind, detail_prefix=''):
            return next((t for t, k, d in evs if k == kind and d.startswith(detail_prefix)), None)

        t_docked = first('state', 'DOCKED')
        durations: dict[str, float] = {}
        state_evs = [(t, d.split(' ')[0]) for t, k, d in evs if k == 'state']
        t_end = self.rows[-1]['t'] if self.rows else 0.0
        for (t_a, s_a), (t_b, _) in zip(state_evs, state_evs[1:] + [(t_end, '')]):
            durations[s_a] = durations.get(s_a, 0.0) + max(0.0, t_b - t_a)

        docked_row = None
        if t_docked is not None:
            docked_row = next((r for r in self.rows if r['t'] >= t_docked), None)
        docked_row = docked_row or (self.rows[-1] if self.rows else None)
        precision = {}
        if docked_row and docked_row['e_lat'] is not None:
            e_lat, e_yaw = docked_row['e_lat'], math.degrees(docked_row['e_yaw'])
            precision = {
                'e_lat_mm': 1000.0 * e_lat, 'e_yaw_deg': e_yaw,
                'e_long_mm': 1000.0 * docked_row['e_long'],
                'lateral_ok': abs(e_lat) <= LATERAL_TOL_M, 'yaw_ok': abs(e_yaw) <= YAW_TOL_DEG,
                'nota': 'estimada por el detector (no ground truth)',
            }
        return {
            'status': status, 'mode': self.mode, 'tag': self.tag, 'wall_start': self.wall_start,
            't_first_detection': first('first_detection'),
            't_predock': first('predock'),
            't_docked': t_docked,
            'within_time_limit': t_docked is not None and t_docked <= TIME_LIMIT_S,
            'duration_total': t_end,
            'state_durations': durations,
            'attempts': attempts,
            'hazards': sum(1 for _, k, _ in evs if k == 'hazard'),
            'precision_at_dock': precision,
            'events': [{'t': round(t, 3), 'kind': k, 'detail': d} for t, k, d in evs],
            'params': self.params,
        }

    def finish(self, status: str, events: list, events_t_start: float | None,
               attempts: int = 0) -> str | None:
        if self.finished or not self.started:
            return None
        self.finished = True
        summary = self.build_summary(
            status, events, events_t_start if events_t_start is not None else self.t0, attempts)
        name = datetime.now().strftime('%Y%m%d_%H%M%S') + f'_{self.mode}'
        if self.tag:
            name += f'_{self.tag}'
        run_dir = os.path.join(self.output_dir, name)
        suffix = 1
        while os.path.exists(run_dir):
            suffix += 1
            run_dir = os.path.join(self.output_dir, f'{name}_{suffix}')
        try:
            os.makedirs(run_dir, exist_ok=True)
            with open(os.path.join(run_dir, 'timeseries.csv'), 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=_COLUMNS)
                writer.writeheader()
                writer.writerows(self.rows)
            with open(os.path.join(run_dir, 'summary.json'), 'w') as f:
                json.dump(summary, f, indent=2, ensure_ascii=False)
        except OSError as exc:
            self._log(f'No se pudo escribir el reporte en {run_dir}: {exc}')
            return None
        if self.save_plot:
            plot_run(self.rows, summary, os.path.join(run_dir, 'run.png'), self._log)
        self._log(f'Reporte de la corrida en {run_dir}')
        return run_dir


def _state_spans(rows):
    spans, start = [], 0
    for i in range(1, len(rows) + 1):
        if i == len(rows) or rows[i]['state'] != rows[start]['state']:
            t_end = rows[i]['t'] if i < len(rows) else rows[-1]['t']
            spans.append((rows[start]['t'], t_end, rows[start]['state']))
            start = i
    return spans


def plot_run(rows: list[dict], summary: dict, path: str,
             log: Callable[[str], None] = print):
    """Gráfica de la corrida (matplotlib opcional, backend sin pantalla)."""
    if not rows:
        return
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        log('matplotlib no está instalado: se omite run.png')
        return

    t = [r['t'] for r in rows]

    def series(key, scale=1.0):
        return [r[key] * scale if r[key] is not None else math.nan for r in rows]

    fig = plt.figure(figsize=(14, 10))
    grid = fig.add_gridspec(4, 2, width_ratios=[2.2, 1], hspace=0.35)
    ax_state = fig.add_subplot(grid[0, 0])
    ax_dist = fig.add_subplot(grid[1, 0], sharex=ax_state)
    ax_err = fig.add_subplot(grid[2, 0], sharex=ax_state)
    ax_cmd = fig.add_subplot(grid[3, 0], sharex=ax_state)
    ax_xy = fig.add_subplot(grid[:, 1])

    for t_a, t_b, state in _state_spans(rows):
        for ax in (ax_state, ax_dist, ax_err, ax_cmd):
            ax.axvspan(t_a, t_b, color=_STATE_COLORS.get(state, '#cccccc'), alpha=0.15, lw=0)
        ax_state.axvspan(t_a, t_b, ymin=0.2, ymax=0.8,
                         color=_STATE_COLORS.get(state, '#cccccc'), alpha=0.9, lw=0)
        if t_b - t_a > 0.03 * max(t[-1], 1e-3):
            ax_state.text((t_a + t_b) / 2, 0.5, state.replace('_', '\n'), ha='center',
                          va='center', fontsize=7, color='white', fontweight='bold')
    ax_state.set_yticks([])
    ax_state.set_title(
        f"{summary['mode']} {summary['tag']} — {summary['status']}  |  pre-dock: "
        f"{_fmt(summary['t_predock'])}  dock: {_fmt(summary['t_docked'])}", fontsize=10)

    for ax in (ax_state, ax_dist, ax_err, ax_cmd):
        for key, color, label in (('t_predock', '#ab47bc', 'pre-dock'),
                                  ('t_docked', '#2e7d32', 'dock')):
            if summary.get(key) is not None:
                ax.axvline(summary[key], color=color, ls='--', lw=1.2)
                if ax is ax_dist:
                    ax.annotate(f'{label} {summary[key]:.1f} s', (summary[key], 1.0),
                                xycoords=('data', 'axes fraction'), rotation=90,
                                va='top', ha='right', fontsize=8, color=color)

    ax_dist.plot(t, series('dist_predock'), label='dist. al pre-dock [m]')
    ax_dist.plot(t, [-x for x in series('e_long')], label='dist. al dock (-e_long) [m]')
    ax_dist.set_ylabel('m')
    ax_dist.legend(fontsize=8, loc='upper right')

    ax_err.plot(t, series('e_lat', 1000.0), label='e_lat [mm]', color='#1565c0')
    ax_err.axhspan(-LATERAL_TOL_M * 1000, LATERAL_TOL_M * 1000, color='#1565c0', alpha=0.08)
    ax_err.set_ylabel('mm')
    ax_err.set_ylim(-60, 60)
    ax_yaw = ax_err.twinx()
    ax_yaw.plot(t, [math.degrees(x) for x in series('e_yaw')], label='e_yaw [°]',
                color='#e65100')
    ax_yaw.axhspan(-YAW_TOL_DEG, YAW_TOL_DEG, color='#e65100', alpha=0.08)
    ax_yaw.set_ylabel('°')
    ax_yaw.set_ylim(-30, 30)
    ax_err.legend(loc='upper left', fontsize=8)
    ax_yaw.legend(loc='upper right', fontsize=8)

    ax_cmd.plot(t, series('v'), label='v [m/s]')
    ax_cmd.plot(t, series('w'), label='w [rad/s]')
    ax_cmd.set_xlabel('t desde el arranque de dock_node [s]')
    ax_cmd.legend(fontsize=8, loc='upper right')

    _plot_xy(ax_xy, rows, summary)
    fig.savefig(path, dpi=110, bbox_inches='tight')
    plt.close(fig)


def _plot_xy(ax, rows, summary):
    goal_row = next((r for r in reversed(rows) if r['goal_x'] is not None), None)
    if goal_row is None:
        ax.set_title('sin detección')
        return
    goal = Pose2D(goal_row['goal_x'], goal_row['goal_y'], goal_row['goal_yaw'])
    # Frame del dock para el dibujo: x = distancia desde la pared hacia la sala, y = lateral.
    offset = summary['params'].get('geometry', {}).get('dock_offset_m', 0.266)
    predock_d = summary['params'].get('geometry', {}).get('predock_distance_m', 0.40)
    xs, ys = [], []
    for r in rows:
        rel = pose_relative(goal, Pose2D(r['x'], r['y'], r['yaw']))
        xs.append(offset - rel.x)
        ys.append(-rel.y)
    ax.plot(ys, xs, color='#1565c0', lw=1.2, label='trayectoria')
    ax.plot(ys[0], xs[0], 'o', color='#1565c0', label='inicio')
    ax.axhline(0.0, color='k', lw=2)
    for sign in (1, -1):
        ax.add_patch(_rect(sign * 0.0475 if sign > 0 else -0.1275, 0.0, 0.08, 0.08))
    ax.plot(0, offset, marker=(3, 0, 180), color='#d32f2f', ms=12, ls='', label='dock real')
    ax.plot(0, offset + predock_d, marker=(3, 0, 180), color='#d81b60', ms=10, ls='',
            label='pre-dock', alpha=0.6)
    ax.set_aspect('equal')
    ax.invert_yaxis()
    ax.set_xlabel('lateral [m]')
    ax.set_ylabel('distancia a la pared [m]')
    prec = summary.get('precision_at_dock') or {}
    if prec:
        ax.set_title(f"al acoplar: e_lat={prec['e_lat_mm']:.1f} mm, "
                     f"e_yaw={prec['e_yaw_deg']:.2f}°", fontsize=9)
    ax.legend(fontsize=7, loc='lower right')


def _rect(y0, x0, width, height):
    from matplotlib.patches import Rectangle
    return Rectangle((y0, x0), width, height, color='#8d6e63', alpha=0.7)


def _fmt(value):
    return f'{value:.1f} s' if value is not None else '—'
