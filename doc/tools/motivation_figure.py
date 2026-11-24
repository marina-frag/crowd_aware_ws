#!/usr/bin/env python3
"""Motivation figure: DWA arcs, fixed DWAL and dynamic DWAL on the same schematic scene.

Paths, collision checks and clusters are computed, not drawn by hand. Geometry follows
Moustris & Tzafestas 2026 (arc-line paths, Sec. 3.2-3.3). The dynamic rule uses the project
values: braking distance at 0.3 m/s, separation 5, span 0.2 m, |dL| > 0.3 m.
"""
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Arc, FancyBboxPatch, Rectangle

R, DS, INFLATE = 1.8, 0.05, 0.17
SEPARATION, MIN_SPAN, TAU = 5, 0.2, 0.3
BRAKE_MARGIN, BRAKE_LENGTH = 0.05, 0.26
K_MAX = 4.0

WALLS = [(-2.35, -0.75, 0.35, 3.0), (0.85, -0.75, 1.45, 1.1), (0.85, 1.45, 1.45, 0.8)]
PERSON = (1.08, 0.78, 0.44, 0.24)
OBSTACLES = WALLS + [PERSON]

INK, MUTED, WALL, ROBOT, PERSON_C = '#20242B', '#A3A9B3', '#E3E5E8', '#5B9BD5', '#F28C28'
CLUSTERS = ['#1D63C9', '#0B8A63', '#C2410C']


def blocked(x, y):
    return any(a - INFLATE <= x <= a + w + INFLATE and b - INFLATE <= y <= b + h + INFLATE
               for a, b, w, h in OBSTACLES)


def arc_point(k, s):
    """Robot at the origin, heading +y, positive curvature turns right."""
    if abs(k) < 1e-9:
        return 0.0, s
    return (1 - math.cos(k * s)) / k, math.sin(k * s) / k


def arc_line_point(phi, s):
    k = 2 * math.sin(phi) / R if abs(phi) <= math.pi / 4 else math.copysign(1 / (R * math.cos(phi)), phi)
    if abs(phi) <= math.pi / 4:
        return arc_point(k, s)
    corner = math.pi / (2 * abs(k))
    if s <= corner:
        return arc_point(k, s)
    return math.copysign(1 / abs(k) + s - corner, phi), 1 / abs(k)


def total_length(phi):
    if abs(phi) <= math.pi / 4:
        k = 2 * math.sin(phi) / R
        return R if abs(k) < 1e-9 else 2 * math.asin(min(abs(k) * R / 2, 1)) / abs(k)
    rho = R * math.cos(phi)
    return math.pi * rho / 2 + math.sqrt(R * R - rho * rho) - rho


def trace(point, length):
    pts, s = [point(0.0)], 0.0
    while s + DS <= length + 1e-9:
        p = point(s + DS)
        if blocked(*p):
            return pts, s, False
        pts.append(p)
        s += DS
    return pts, s, True


def bundle():
    phi_max = math.acos(1 / (K_MAX * R))
    step = 2 * math.asin(0.05 / (2 * R))
    count = int(phi_max / step)
    out = []
    for i in range(-count, count + 1):
        phi = i * step
        pts, s, free = trace(lambda d: arc_line_point(phi, d), total_length(phi))
        k = abs(2 * math.sin(phi) / R) if abs(phi) <= math.pi / 4 else 1 / (R * math.cos(phi))
        out.append(dict(i=i, phi=phi, pts=pts, L=s, free=free, k=k))
    return out


def clusters(paths, accepted, use_jump):
    groups, now = [], []
    for p in paths:
        if not accepted(p):
            continue
        if now and (p['i'] - now[-1]['i'] > SEPARATION or (use_jump and abs(p['L'] - now[-1]['L']) > TAU)):
            groups.append(now)
            now = []
        now.append(p)
    if now:
        groups.append(now)
    return [g for g in groups if 2 * R * math.sin(abs(g[-1]['phi'] - g[0]['phi']) / 2) >= MIN_SPAN]


def dynamic_accept(p):
    limit = p['L'] if p['k'] < 1e-9 else min(p['L'], BRAKE_MARGIN + math.pi / (2 * p['k']))
    return p['free'] or limit >= BRAKE_LENGTH


def draw_scene(ax, title):
    ax.set_xlim(-2.44, 2.40)
    ax.set_ylim(-1.70, 2.86)
    ax.set_aspect('equal')
    ax.axis('off')
    ax.text(-0.02, 2.40, title, ha='center', va='bottom', fontsize=22, fontweight='bold', color=INK)
    for a, b, w, h in WALLS:
        ax.add_patch(Rectangle((a, b), w, h, fc=WALL, ec=INK, lw=2.0, zorder=1))
    a, b, w, h = PERSON
    ax.add_patch(FancyBboxPatch((a, b), w, h, boxstyle='round,pad=0,rounding_size=0.06',
                                fc=PERSON_C, ec=INK, lw=1.8, zorder=5))
    ax.text(a + w / 2, b + h / 2, 'human', ha='center', va='center', fontsize=7.8, color=INK,
            fontweight='bold', zorder=6)
    ax.add_patch(Arc((0, 0), 2 * R, 2 * R, theta1=0, theta2=180, ls=(0, (2, 3)), lw=1.2, color='#6B7280', zorder=2))
    ax.add_patch(FancyBboxPatch((-0.3, -0.62), 0.6, 0.56, boxstyle='round,pad=0,rounding_size=0.05',
                                fc=ROBOT, ec='#2F6DA8', lw=2, zorder=7))
    for side in (-1, 1):
        ax.add_patch(Rectangle((side * 0.335 - 0.035, -0.47), 0.07, 0.26, fc='#3D79B5', ec='#2F6DA8', lw=1, zorder=6))
    ax.plot([0], [0], 'o', ms=4.5, color=INK, zorder=8)
    ax.annotate('requested\ndirection', xy=(1.30, 0.70), xytext=(1.57, -0.20), ha='center', va='center',
                fontsize=11, color='#6D28D9', fontweight='bold', zorder=9,
                arrowprops=dict(arrowstyle='-|>', lw=2.2, color='#7C3AED', mutation_scale=15))


def line(ax, pts, **style):
    ax.plot([p[0] for p in pts], [p[1] for p in pts], solid_capstyle='round', **style)


def caption(ax, head, body, color):
    ax.text(-0.02, -1.04, head, ha='center', va='center', fontsize=15, fontweight='bold', color=color)
    ax.text(-0.02, -1.42, body, ha='center', va='center', fontsize=12, color=INK, linespacing=1.3)


def panel_dwa(ax):
    draw_scene(ax, 'DWA')
    for k in [i * 0.17 for i in range(-10, 11)]:
        pts, _, free = trace(lambda s: arc_point(k, s), R)
        if free:
            line(ax, pts, color=CLUSTERS[0], lw=2.0, zorder=3)
        else:
            line(ax, pts, color=MUTED, lw=1.5, ls=(0, (3, 2)), zorder=3)
            ax.plot(*pts[-1], marker='x', ms=7, mew=2, color='#B91C1C', zorder=4)
    caption(ax, 'Arcs hide openings', 'Arcs hit walls, so valid\ndirections are discarded', '#B91C1C')


def draw_clusters(ax, groups):
    for c, group in enumerate(groups):
        color = CLUSTERS[c % len(CLUSTERS)]
        index = sorted(set(range(0, len(group), 3)) | {len(group) - 1})
        for j in index:
            edge = j in (0, len(group) - 1)
            line(ax, group[j]['pts'], color=color, lw=2.6 if edge else 1.25, alpha=1 if edge else .72, zorder=3 + edge)
            if not group[j]['free'] and edge:
                ax.plot(*group[j]['pts'][-1], 'o', ms=5.5, color=color, zorder=5)


def panel_fixed(ax, paths):
    draw_scene(ax, 'Fixed DWAL')
    for p in paths[::4]:
        if not p['free']:
            line(ax, p['pts'], color=MUTED, lw=1.1, ls=(0, (3, 2)), alpha=.75, zorder=2)
    groups = clusters(paths, lambda p: p['free'], False)
    draw_clusters(ax, groups)
    caption(ax, 'Too strict near people', 'Every shortened path is discarded,\nso usable free space is lost', '#B45309')
    return groups


def panel_dynamic(ax, paths):
    draw_scene(ax, 'Dynamic DWAL')
    for p in paths[::4]:
        if not dynamic_accept(p):
            line(ax, p['pts'], color=MUTED, lw=1.1, ls=(0, (3, 2)), alpha=.75, zorder=2)
    groups = clusters(paths, dynamic_accept, True)
    draw_clusters(ax, groups)
    caption(ax, 'Keeps safe short motion', 'Paths that can brake in time\nstay as separate options', '#0B8A63')
    return groups


def main():
    paths = bundle()
    fig, axes = plt.subplots(1, 3, figsize=(16, 5.2))
    panel_dwa(axes[0])
    fixed = panel_fixed(axes[1], paths)
    dynamic = panel_dynamic(axes[2], paths)
    for ax in axes:
        ax.add_patch(Rectangle((-2.42, -1.68), 4.80, 4.52, fill=False, ec=INK, lw=1.6, clip_on=False))
    fig.subplots_adjust(left=0.008, right=0.992, top=0.985, bottom=0.015, wspace=0.03)
    out = Path(__file__).resolve().parents[1] / 'assets' / 'motivation_dwa_dwal'
    fig.savefig(out.with_suffix('.png'), dpi=220, facecolor='white')
    fig.savefig(out.with_suffix('.svg'), facecolor='white')
    describe = lambda gs: [(len(g), round(min(p['L'] for p in g), 2), round(max(p['L'] for p in g), 2)) for g in gs]
    print('paths', len(paths), 'free', sum(p['free'] for p in paths))
    print('fixed clusters (n, min L, max L)', describe(fixed))
    print('dynamic clusters (n, min L, max L)', describe(dynamic))


if __name__ == '__main__':
    main()
