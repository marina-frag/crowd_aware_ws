"""Seed variation of every scenario except dense_crowd / dense_static: one figure per
scenario (doc/scenario_seeds/<name>.png), one panel per seed, using prepare_scene.vary_agents
(the same code the experiments use). Robot reference path (green, one figure per path shape: <name>_<shape>.png), agents at start (red,
r = agent radius), routes (blue dashed), seed-1 start positions (grey), walls (grey boxes).
    python3 doc/tools/plot_other_scenarios.py [first_seed last_seed]   (default 1 9)"""
import math, re, sys, xml.etree.ElementTree as ET
from pathlib import Path
import yaml, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon

PKG = Path(__file__).resolve().parents[2] / 'src/crowd_aware_simulation'
sys.path.insert(0, str(PKG / 'scripts'))
import types; sys.modules.setdefault('xacro', types.ModuleType('xacro'))  # only build_path is used
try:
    from prepare_scene import build_path, vary_agents
except Exception as e:
    sys.exit(f'prepare_scene import failed: {e}')
NAMES = ['open_area', 'crossing', 'group_blocking', 'doorway', 'narrow_corridor', 'junction', 'cafe']

def boxes(world):
    """(x, y, yaw, sx, sy) of every static box in the world file; includes as 0.8 m squares."""
    out = []
    root = ET.parse(PKG / 'worlds' / world).getroot()
    for m in root.iter('model'):
        p = [float(v) for v in (m.findtext('pose') or '0 0 0 0 0 0').split()]
        for link in m.iter('link'):
            lp = [float(v) for v in (link.findtext('pose') or '0 0 0 0 0 0').split()]
            for col in link.iter('collision'):
                s = col.find('.//box/size')
                if s is None: continue
                cp = [float(v) for v in (col.findtext('pose') or '0 0 0 0 0 0').split()]
                sx, sy, _ = map(float, s.text.split())
                yaw = p[5] + lp[5] + cp[5]
                c, si = math.cos(p[5]), math.sin(p[5])
                ox, oy = lp[0] + cp[0], lp[1] + cp[1]
                out.append((p[0] + c*ox - si*oy, p[1] + si*ox + c*oy, yaw, sx, sy))
    for inc in root.iter('include'):
        uri = inc.findtext('uri') or ''
        if 'ground_plane' in uri or uri.endswith('/cafe'): continue
        p = [float(v) for v in (inc.findtext('pose') or '0 0 0 0 0 0').split()]
        out.append((p[0], p[1], p[5], 0.9, 0.9))
    return out

def rect(x, y, yaw, sx, sy):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c*dx - s*dy, y + s*dx + c*dy) for dx, dy in
            [(-sx/2, -sy/2), (sx/2, -sy/2), (sx/2, sy/2), (-sx/2, sy/2)]]

def paths(sc):
    r = sc['robot']; out = {'line': r['path_xy']}
    if build_path and r.get('path_shapes'):
        try:
            out = {k: [list(map(float, q)) for q in build_path(v, (*map(float, r['path_xy'][0]), 0.0))]
                   for k, v in r['path_shapes'].items()}
        except Exception as e:
            print('path_shapes:', e)
    return out

SEEDS = list(range(int(sys.argv[1]), int(sys.argv[2]) + 1)) if len(sys.argv) > 2 else list(range(1, 10))
OUT = Path(__file__).resolve().parents[1] / 'scenario_seeds'; OUT.mkdir(exist_ok=True)

def draw(ax, pts, params, base, walls):
    for b in walls:
        ax.add_patch(Polygon(rect(*b), closed=True, fc='0.8', ec='0.5'))
    xs, ys = zip(*pts)
    ax.plot(xs, ys, color='tab:green', lw=3, zorder=5)
    ax.plot(xs[0], ys[0], 'o', color='tab:green', ms=8, zorder=6)
    ax.annotate('', (xs[-1], ys[-1]), (xs[-2], ys[-2]), zorder=6,
                arrowprops=dict(arrowstyle='-|>', color='tab:green', lw=3, mutation_scale=20))
    for a in params['agents']:
        d, b0 = params[a], base[a]['init_pose']
        ip = d['init_pose']
        ax.add_patch(Circle((b0['x'], b0['y']), d.get('radius', 0.4), fill=False, ec='0.6', lw=0.8, ls=':'))
        route = [(ip['x'], ip['y'])] + [(d[g]['x'], d[g]['y']) for g in d.get('goals', [])]
        if d.get('cyclic_goals') and len(route) > 2: route.append(route[1])
        xs, ys = zip(*route)
        if len(route) > 1: ax.plot(xs, ys, ls='--', color='tab:blue', lw=1)
        ax.add_patch(Circle((ip['x'], ip['y']), d.get('radius', 0.4), fill=False, ec='red', lw=1.2))
        ax.annotate(f"{a}\nv={d['behavior']['vel']:.2f}", (ip['x'], ip['y']), fontsize=6, ha='center',
                    va='bottom', xytext=(0, 9), textcoords='offset points', color='darkred')
    ax.set_aspect('equal', adjustable='datalim'); ax.grid(alpha=.3); ax.autoscale_view()

for name in NAMES:
    sc = yaml.safe_load(open(PKG / f'config/scenarios/{name}.yaml'))['scenario']
    doc = yaml.safe_load(open(PKG / 'scenarios' / sc['agents']))
    base = doc['hunav_loader']['ros__parameters']
    walls = boxes(sc['world']); start = (*map(float, sc['robot']['path_xy'][0]), 0.0)
    for shape, pts in paths(sc).items():
        n = len(SEEDS); cols = 3; rows = math.ceil(n / cols)
        fig, axes = plt.subplots(rows, cols, figsize=(5 * cols, 4.6 * rows), squeeze=False)
        axes = axes.ravel()
        for ax, seed in zip(axes, SEEDS):
            var = vary_agents(doc, sc['seed_variation'], seed, start) if sc.get('seed_variation') else doc
            draw(ax, pts, var['hunav_loader']['ros__parameters'], base, walls)
            ax.set_title(f'seed {seed}' + (' (baseline, unchanged)' if seed == sc.get('seed_variation', {}).get('baseline_seed', 1) else ''), fontsize=10)
        for ax in axes[n:]: ax.axis('off')
        fig.suptitle(f"{name} / robot path: {shape} (green, start dot -> goal arrow)\n"
                     "agents per seed (red, v = speed m/s), seed-1 start (grey dotted), agent routes (blue dashed)", fontsize=11)
        fig.tight_layout(); f = OUT / f'{name}_{shape}.png'; fig.savefig(f, dpi=100); plt.close(fig); print(f)
