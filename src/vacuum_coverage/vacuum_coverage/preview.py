"""Offline plan preview: render the coverage plan for a map to a PNG.

    python3 -m vacuum_coverage.preview --map maps/living_room.yaml --out plan.png
    python3 -m vacuum_coverage.preview --map ... --start -2.0 -0.5 --simulate

``--simulate`` also drives an approximation of Nav2's pure pursuit along the plan
(see sim.py) and reports coverage and the smallest clearance to obstacles.
Needs matplotlib.
"""

import argparse

import numpy as np

from . import core, sim, validate


def render(grid, plan, out_path, trails=None, tracker=None, title=''):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    res = grid.resolution
    h, w = grid.data.shape
    extent = (grid.origin_x, grid.origin_x + w * res, grid.origin_y, grid.origin_y + h * res)
    img = np.full((h, w, 3), 0.80)                   # unknown
    img[grid.data == 0] = (1.0, 1.0, 1.0)
    img[grid.data == 100] = (0.15, 0.15, 0.15)
    if tracker is not None:
        cov = tracker.covered & (grid.data == 0)
        img[cov] = (0.78, 0.92, 0.80)
        miss = tracker.coverable & ~tracker.covered
        img[miss] = (0.98, 0.78, 0.78)

    fig, ax = plt.subplots(figsize=(8, 8), dpi=110)
    ax.imshow(img, origin='lower', extent=extent, interpolation='nearest')
    wall_c, lane_c = '#d9480f', '#1971c2'
    first = {'wall': True, 'lane': True}
    for piece in plan.pieces:
        xs = [p[0] for p in piece.poses]
        ys = [p[1] for p in piece.poses]
        col = wall_c if piece.kind == 'wall' else lane_c
        label = None
        if first[piece.kind]:
            label = 'wall pass' if piece.kind == 'wall' else 'lanes'
            first[piece.kind] = False
        ax.plot(xs, ys, '-', color=col, lw=1.3, label=label, alpha=0.9)
        ax.plot(xs[0], ys[0], 'o', color=col, ms=3.5)
    if trails:
        for tr in trails:
            a = np.asarray(tr)
            ax.plot(a[::5, 0], a[::5, 1], '-', color='black', lw=0.5, alpha=0.5)
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.legend(loc='upper right', fontsize=8)
    if title:
        ax.set_title(title, fontsize=10)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map', required=True, help='map yaml (nav2_map_server format)')
    ap.add_argument('--out', default='coverage_plan.png')
    ap.add_argument('--start', nargs=2, type=float, default=[-2.0, -0.5], metavar=('X', 'Y'))
    ap.add_argument('--simulate', action='store_true')
    for name, default in vars(core.CoverageParams()).items():
        if isinstance(default, bool):
            continue
        ap.add_argument('--' + name.replace('_', '-'), type=type(default), default=default)
    ap.add_argument('--no-wall-pass', action='store_true')
    ap.add_argument('--no-lanes', action='store_true')
    ap.add_argument('--no-auto-margin', action='store_true',
                    help='with --simulate: do not widen wall_margin until the simulated clearance is safe')
    args = ap.parse_args(argv)

    kwargs = {k: getattr(args, k) for k in vars(core.CoverageParams())
              if not isinstance(getattr(core.CoverageParams(), k), bool)}
    params = core.CoverageParams(wall_pass=not args.no_wall_pass, lanes=not args.no_lanes, **kwargs)
    grid = core.load_map_yaml(args.map)
    start = tuple(args.start)
    trails, tracker, title = None, None, ''
    if args.simulate and not args.no_auto_margin:
        plan, _ = validate.plan_validated(grid, params, start)
        params = plan.params
    else:
        plan = core.plan_coverage(grid, params, start)
    for w_ in plan.warnings:
        print('warning:', w_)
    print('plan:', plan.stats())
    print('wall margin used: %.2f m' % plan.params.wall_margin)

    if args.simulate:
        trails, tracker, clr, cov_free, total, failures = validate.evaluate(
            grid, plan, (start[0], start[1], 0.0), sim.PursuitParams())
        print('simulated: ~%.1f min, failed pieces %d' % (total / 60.0, failures))
        print('  coverage of reachable floor : %.1f %%' % tracker.percent())
        print('  coverage of all free floor  : %.1f %% (cells the robot cannot fit into are excluded above)' % cov_free)
        print('  min clearance to obstacles  : %.3f m (robot radius %.4f m) -> %s'
              % (clr, params.robot_radius, 'OK' if clr >= params.robot_radius + 0.02 else 'TOO CLOSE'))
        title = ('coverage %.1f %% of reachable (%.1f %% of all free floor)   clearance %.3f m   ~%.1f min'
                 % (tracker.percent(), cov_free, clr, total / 60.0))
    render(grid, plan, args.out, trails, tracker, title)
    print('wrote', args.out)


if __name__ == '__main__':
    main()
