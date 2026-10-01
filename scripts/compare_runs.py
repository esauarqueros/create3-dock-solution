#!/usr/bin/env python3
"""
Compara corridas guardadas por el reporte (debug.run_report / run_report:=true).

Uso:
    python3 scripts/compare_runs.py /ws/dock_runs            # tabla por modo+tag
    python3 scripts/compare_runs.py /ws/dock_runs --plot comparacion.png
    python3 scripts/compare_runs.py /ws/dock_runs --group mode

Solo lee los summary.json; no depende de ROS.
"""
import argparse
import glob
import json
import os
import statistics


def _load(root):
    runs = []
    for path in sorted(glob.glob(os.path.join(root, '**', 'summary.json'), recursive=True)):
        with open(path) as f:
            summary = json.load(f)
        summary['_dir'] = os.path.basename(os.path.dirname(path))
        runs.append(summary)
    return runs


def _mean_max(values):
    values = [v for v in values if v is not None]
    if not values:
        return '—'
    return f'{statistics.mean(values):.1f} / {max(values):.1f}'


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('root', help='carpeta con las corridas (p.ej. /ws/dock_runs)')
    parser.add_argument('--group', choices=['mode', 'tag', 'mode+tag'], default='mode+tag')
    parser.add_argument('--plot', default='', help='guardar un gráfico comparativo (PNG)')
    args = parser.parse_args()

    runs = _load(args.root)
    if not runs:
        print(f'No hay summary.json en {args.root}')
        return

    def key(run):
        if args.group == 'mode':
            return run['mode']
        if args.group == 'tag':
            return run['tag'] or '(sin tag)'
        return f"{run['mode']} {run['tag']}".strip()

    groups: dict[str, list] = {}
    for run in runs:
        groups.setdefault(key(run), []).append(run)

    header = (f"{'grupo':<28} {'éxito':>7} {'t_predock med/máx':>18} {'t_dock med/máx':>16} "
              f"{'|e_lat| mm med/máx':>19} {'|e_yaw|° med/máx':>17} {'reintentos':>10}")
    print(header)
    print('-' * len(header))
    for name, items in groups.items():
        ok = [r for r in items if r['t_docked'] is not None]
        prec = [r['precision_at_dock'] for r in ok if r.get('precision_at_dock')]
        print(f'{name:<28} {len(ok):>3}/{len(items):<3} '
              f"{_mean_max([r['t_predock'] for r in ok]):>18} "
              f"{_mean_max([r['t_docked'] for r in ok]):>16} "
              f"{_mean_max([abs(p['e_lat_mm']) for p in prec]):>19} "
              f"{_mean_max([abs(p['e_yaw_deg']) for p in prec]):>17} "
              f"{sum(r['attempts'] for r in items):>10}")
    print('\nPrecisión estimada por el detector (no ground truth).')

    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        names = list(groups)
        fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))
        data = [
            ('t hasta el dock [s]', [[r['t_docked'] for r in groups[n] if r['t_docked']]
                                     for n in names]),
            ('t hasta el pre-dock [s]', [[r['t_predock'] for r in groups[n] if r['t_predock']]
                                         for n in names]),
            ('|e_lat| al acoplar [mm]', [[abs(r['precision_at_dock']['e_lat_mm'])
                                          for r in groups[n] if r.get('precision_at_dock')]
                                         for n in names]),
        ]
        for ax, (title, values) in zip(axes, data):
            ax.boxplot([v if v else [float('nan')] for v in values], labels=names)
            ax.set_title(title)
            ax.tick_params(axis='x', rotation=30)
        axes[0].axhline(45.0, color='g', ls='--', lw=1, label='45 s = puntaje completo')
        axes[0].legend(fontsize=8)
        axes[2].axhline(8.0, color='r', ls='--', lw=1, label='~8 mm (±6°)')
        axes[2].legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(args.plot, dpi=110)
        print(f'Gráfico en {args.plot}')


if __name__ == '__main__':
    main()
