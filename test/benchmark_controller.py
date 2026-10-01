#!/usr/bin/env python3
"""
Benchmark offline del controller: compara los modos de aproximación en el simulador uniciclo.

Uso (desde la raíz del paquete):
    PYTHONPATH=. python3 test/benchmark_controller.py
    PYTHONPATH=. python3 test/benchmark_controller.py --noise-pos 0.003 --noise-yaw 0.3
    PYTHONPATH=. python3 test/benchmark_controller.py --set final.v=0.10 approach.v_max=0.30
    PYTHONPATH=. python3 test/benchmark_controller.py --save-dir /ws/dock_runs/bench

No es un test de CI (no empieza por test_). Imprime tasa de éxito, tiempos
hasta el pre-dock y el dock, error final (verdadero, del simulador) y holgura
mínima a las cajas.
"""
import argparse
import math
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from create3_dock_solution.control.params import APPROACH_MODES, ControllerParams  # noqa: E402
from create3_dock_solution.diagnostics.run_recorder import RunRecorder  # noqa: E402
from sim_world import evaluation_grid, run_episode, SimConfig  # noqa: E402


def _apply_overrides(params: ControllerParams, overrides: list[str]) -> ControllerParams:
    nested = params.to_nested_dict()
    for item in overrides:
        key, value = item.split('=', 1)
        group, name = key.split('.', 1)
        current = nested[group][name]
        nested[group][name] = type(current)(value) if not isinstance(current, bool) \
            else value.lower() in ('1', 'true', 'yes')
    return ControllerParams.from_dict(nested)


def _stats(values):
    if not values:
        return '   —   '
    return f'{statistics.mean(values):5.1f} / {max(values):5.1f}'


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--modes', nargs='+', default=list(APPROACH_MODES))
    parser.add_argument('--nx', type=int, default=4)
    parser.add_argument('--ny', type=int, default=5)
    parser.add_argument('--nyaw', type=int, default=6)
    parser.add_argument('--noise-pos', type=float, default=0.002)
    parser.add_argument('--noise-yaw', type=float, default=0.2)
    parser.add_argument('--set', nargs='*', default=[], metavar='GRUPO.PARAM=VALOR')
    parser.add_argument('--save-dir', default='', help='guardar reporte por corrida (lento)')
    args = parser.parse_args()

    poses = evaluation_grid(args.nx, args.ny, args.nyaw)
    print(f'{len(poses)} poses del rango de evaluación, ruido {args.noise_pos * 1000:.1f} mm '
          f'/ {args.noise_yaw:.2f}°\n')
    header = (f"{'modo':<12} {'éxito':>7} {'t_predock media/máx':>20} {'t_dock media/máx':>18} "
              f"{'|e_lat| mm med/máx':>19} {'|e_yaw|° med/máx':>17} {'holgura mín cm':>15} "
              f"{'reintentos':>10}")
    print(header)
    print('-' * len(header))
    for mode in args.modes:
        params = _apply_overrides(ControllerParams(), args.set + [f'approach.mode={mode}'])
        results = []
        for i, pose in enumerate(poses):
            cfg = SimConfig(noise_pos=args.noise_pos, noise_yaw_deg=args.noise_yaw, seed=i)
            recorder = None
            if args.save_dir:
                recorder = RunRecorder(args.save_dir, tag=f'bench{i:03d}', log=lambda _m: None)
            results.append(run_episode(pose, params, cfg, recorder))
        ok = [r for r in results if r.docked]
        print(f'{mode:<12} {len(ok):>3}/{len(results):<3} '
              f'{_stats([r.t_predock for r in ok if r.t_predock is not None]):>20} '
              f'{_stats([r.t_docked for r in ok]):>18} '
              f'{_stats([abs(r.e_lat) * 1000 for r in ok]):>19} '
              f'{_stats([abs(r.e_yaw_deg) for r in ok]):>17} '
              f'{min(r.min_clearance for r in results) * 100:>15.1f} '
              f'{sum(r.attempts for r in results):>10}')
        failures = [(p, r) for p, r in zip(poses, results) if not r.docked]
        for pose, r in failures[:5]:
            print(f'   fallo desde ({pose.x:.2f}, {pose.y:.2f}, {math.degrees(pose.yaw):.0f}°): '
                  f'estado {r.state}, intentos {r.attempts}')


if __name__ == '__main__':
    main()
