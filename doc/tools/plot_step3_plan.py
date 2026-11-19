"""Step-3 plan: what each run looks like (input2.md decisions M, N).
One panel per (scenario, path): medium density, seed 1 agents (red circles, routes blue dashed),
start positions of seeds 2-10 (small red dots), robot reference path (green, start dot -> goal
arrow), walls / tables (grey). Paths are read from the scenario yaml files (path_shapes). Prints the clearance of every path to walls / tables.
    python3 doc/tools/plot_step3_plan.py      -> doc/step3_plan.png"""
import math, sys, types, xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np, yaml, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon

REPO = Path(__file__).resolve().parents[2]
PKG = REPO/'src/crowd_aware_simulation'
sys.path.insert(0, str(PKG/'scripts'))
sys.modules.setdefault('xacro', types.ModuleType('xacro'))  # only the pure-python helpers are used
from prepare_scene import build_path, path_start_pose, vary_agents, layout_agents

HALF_WIDTH = 0.308  # DWAL footprint 0.724 x 0.616 m (dwal_params.yaml)
SEEDS = range(1, 11)


# Step 3 conditions, as in run_experiments.sh (CONDITIONS); paths come from the scenario yaml files.
CONDITIONS = ['dense_crowd:hard_curve', 'dense_static:hard_curve', 'dense_static:hard_curve_1m', 'open_area:hard_curve', 'crossing:line',
              'group_blocking:line', 'group_blocking:detour_left', 'doorway:side_approach', 'doorway:u_turn', 'narrow_corridor:keep_right',
              'junction:left_turn', 'junction:straight_right', 'cafe:between_tables']
CAFE_WALLS = [(-5.03, 0.53, 0, .12, 23.16), (4.24, -.31, 0, .12, 21.48), (-1.48, 7.43, 0, 7.2, .12),
              (2.09, 8.9, 0, .12, 3.05), (3.155, 10.4, 0, 2.13, .12), (0.615, -10.98, 0, 7.36, .15),
              (-4.62, -10.98, 0, .93, .15), (-4.385, 0.26, 0, 1.03, 1.03)]  # gazebo model 'cafe'


def boxes(world):
    out = []
    root = ET.parse(PKG/'worlds'/world).getroot()
    for m in root.iter('model'):
        p = [float(v) for v in (m.findtext('pose') or '0 0 0 0 0 0').split()]
        for col in m.iter('collision'):
            s = col.find('.//box/size')
            if s is not None:
                sx, sy, _ = map(float, s.text.split()); out.append((p[0], p[1], p[5], sx, sy))
    for inc in root.iter('include'):
        uri = inc.findtext('uri') or ''
        p = [float(v) for v in (inc.findtext('pose') or '0 0 0 0 0 0').split()]
        if uri.endswith('/cafe'): out += CAFE_WALLS
        elif 'cafe_table' in uri: out.append((p[0], p[1], p[5], 0.913, 0.913))
    return out


def rect(x, y, yaw, sx, sy):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c*dx - s*dy, y + s*dx + c*dy) for dx, dy in [(-sx/2, -sy/2), (sx/2, -sy/2), (sx/2, sy/2), (-sx/2, sy/2)]]


def clearance(path, walls):
    """Smallest distance from the robot's side (path point +- half width) to any box, ignoring
    far-away landmarks (boxes > 5 m from the whole path)."""
    best = math.inf
    for x, y, yaw, sx, sy in walls:
        c, s = math.cos(-yaw), math.sin(-yaw)
        for px, py in path:
            lx, ly = c*(px-x) - s*(py-y), s*(px-x) + c*(py-y)
            dx, dy = max(abs(lx) - sx/2, 0), max(abs(ly) - sy/2, 0)
            best = min(best, math.hypot(dx, dy) - HALF_WIDTH)
    return best


def panels():
    out = []
    for cond in CONDITIONS:
        name, shape = cond.split(':')
        sc = yaml.safe_load(open(PKG/f'config/scenarios/{name}.yaml'))['scenario']
        doc = yaml.safe_load(open(PKG/'scenarios'/sc['agents']))
        spec = (sc['robot'].get('path_shapes') or {}).get(shape)
        if spec is None:  # line: the scenario's own path_xy
            path = [list(map(float, q)) for q in sc['robot']['path_xy']]; start = (0.0, 0.0, 0.0)
        else:
            path = build_path(spec, (0.0, 0.0, 0.0))
            start = tuple(path_start_pose(path)) if spec['type'] == 'points' else (0.0, 0.0, 0.0)
        if sc.get('agent_layout'):
            agents = [layout_agents(doc, sc['agent_layout'], 'medium', path, shape, seed) for seed in SEEDS]
        else:
            agents = [vary_agents(doc, sc['seed_variation'], seed, start) for seed in SEEDS]
        out.append((name, shape, sc['world'], path, agents))
    return out


def main():
    items = panels()
    cols = 3; rows = math.ceil(len(items) / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(6 * cols, 5.2 * rows), squeeze=False); axes = axes.ravel()
    print(f'{"scenario / path":40} clearance to walls/tables [m]')
    for ax, (name, label, world, path, agents) in zip(axes, items):
        walls = boxes(world)
        for b in walls: ax.add_patch(Polygon(rect(*b), closed=True, fc='0.8', ec='0.5'))
        base = agents[0]['hunav_loader']['ros__parameters']
        for a in base['agents']:
            d = base[a]; ip = d['init_pose']
            route = [(ip['x'], ip['y'])] + [(d[g]['x'], d[g]['y']) for g in d.get('goals', [])]
            if len(route) > 1: ax.plot(*zip(*route), ls='--', color='tab:blue', lw=0.9)
            ax.add_patch(Circle((ip['x'], ip['y']), d.get('radius', 0.4), fill=False, ec='red', lw=1.1))
            for doc in agents[1:]:
                if a not in doc['hunav_loader']['ros__parameters']: continue  # skipped in a tight bend for this seed
                q = doc['hunav_loader']['ros__parameters'][a]['init_pose']
                ax.plot(q['x'], q['y'], '.', color='red', ms=3, alpha=0.6)
        xs, ys = zip(*path)
        ax.plot(xs, ys, color='tab:green', lw=3, zorder=5)
        ax.plot(xs[0], ys[0], 'o', color='tab:green', ms=8, zorder=6)
        ax.annotate('', (xs[-1], ys[-1]), (xs[max(-3, -len(xs))], ys[max(-3, -len(ys))]), zorder=6,
                    arrowprops=dict(arrowstyle='-|>', color='tab:green', lw=3, mutation_scale=20))
        near = [w for w in walls if min(math.hypot(w[0]-x, w[1]-y) for x, y in path) < 5]
        cl = clearance(path, near) if near else math.inf
        tag = f'{name} / {label}'
        print(f'{tag:40} {cl:.2f}' if near else f'{tag:40} no walls nearby')
        n = len(base['agents'])
        ax.set_title(f'{tag}: {n} agents' + (f', clearance {cl:.2f} m' if near else ''), fontsize=10)
        if name == 'cafe': ax.set_xlim(-5.5, 4.8); ax.set_ylim(-11.5, 8)
        ax.set_aspect('equal', adjustable='datalim'); ax.grid(alpha=.3)
        if name != 'cafe': ax.autoscale_view()
    for ax in axes[len(items):]: ax.axis('off')
    fig.suptitle('Step 3 plan (medium density, seeds 1-10, 2 methods each): robot path (green), agents seed 1 (red circles, '
                 'routes blue dashed), start of seeds 2-10 (red dots), walls / tables (grey)', fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.98)); out = REPO/'doc'/'step3_plan.png'; fig.savefig(out, dpi=90); print(out)


if __name__ == '__main__':
    main()
