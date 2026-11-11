"""Path deviation of the executed trajectory from the desired reference path.

Autonomous runs only (the reference controller follows a known path). Definitions:
  cross-track error  distance from the executed trajectory (resampled every STEP m of
                     travelled arc length, so standing still does not weigh in) to the
                     desired polyline: mean, RMS, p95, max.
  ADE / FDE          mean / final distance between the executed and the desired path at
                     EQUAL ARC LENGTH, both measured from the first pose of the window
                     (the desired path from that pose's projection on it; the recording
                     may start after the robot has moved), over the common length. Same
                     definition as the SCAND ADE/FDE (offline_metrics.intent_metrics),
                     where the demonstrator path takes the place of the desired path.
  progress           arc length of the final pose projected on the desired path / its length.
  goal reached       the final pose within goal_tolerance_m of the path end;
                     time_to_goal_s from the window start to the first such pose.
"""
import math

STEP = 0.05


def resample(points, step=STEP):
    """Points every `step` of arc length along a polyline; returns (s, points)."""
    import numpy as np
    p = np.asarray(points, dtype=float)
    if len(p) < 2:
        return np.zeros(len(p)), p
    seg = np.hypot(*np.diff(p, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    grid = np.arange(0.0, s[-1] + 1e-9, step)
    return grid, np.stack([np.interp(grid, s, p[:, 0]), np.interp(grid, s, p[:, 1])], axis=1)


def project(points, q):
    """(distance, arc length of the closest point) of q on the polyline points."""
    import numpy as np
    a, b = points[:-1], points[1:]
    ab = b - a
    t = np.clip(((q - a) * ab).sum(1) / np.maximum((ab * ab).sum(1), 1e-12), 0.0, 1.0)
    d = np.hypot(*(a + ab * t[:, None] - q).T)
    i = int(np.argmin(d))
    seg = np.hypot(*ab.T)
    return float(d[i]), float(seg[:i].sum() + t[i] * seg[i])


def equal_arc_errors(desired, actual, step=STEP):
    """Distances between two paths at equal arc length (common length); the desired path
    starts at the projection of the first actual pose."""
    import numpy as np
    s_d, a = resample(desired, step)
    _, s0 = project(np.asarray(desired, dtype=float), np.asarray(actual[0], dtype=float))
    a = a[s_d >= s0 - 1e-9]
    _, b = resample(actual, step)
    n = min(len(a), len(b))
    return np.hypot(*(a[:n] - b[:n]).T)


def path_deviation(desired, odom, start, end, goal_tolerance):
    """desired: [[x, y], ...]; odom: [(t, x, y, ...)]; window [start, end]."""
    import numpy as np
    rows = [o for o in odom if (start is None or o[0] >= start) and (end is None or o[0] <= end)]
    if len(rows) < 2 or len(desired) < 2:
        return {'available': False, 'reason': 'fewer than two poses in the window or no desired path'}
    ref = np.asarray(desired, dtype=float)
    traj = np.array([(o[1], o[2]) for o in rows])
    _, trail = resample(traj)
    cte = np.array([project(ref, q)[0] for q in trail]) if len(trail) else np.zeros(0)
    errs = equal_arc_errors(ref, traj)
    ref_len = float(np.hypot(*np.diff(ref, axis=0).T).sum())
    act_len = float(np.hypot(*np.diff(traj, axis=0).T).sum())
    end_dist = [math.dist((o[1], o[2]), ref[-1]) for o in rows]
    reached = next((o[0] for o, d in zip(rows, end_dist) if d <= goal_tolerance), None)
    _, s_final = project(ref, traj[-1])
    return {
        'available': True,
        'cross_track_mean_m': float(cte.mean()) if len(cte) else None,
        'cross_track_rms_m': float(np.sqrt((cte ** 2).mean())) if len(cte) else None,
        'cross_track_p95_m': float(np.percentile(cte, 95)) if len(cte) else None,
        'cross_track_max_m': float(cte.max()) if len(cte) else None,
        'ade_m': float(errs.mean()) if len(errs) else None,
        'fde_m': float(errs[-1]) if len(errs) else None,
        'desired_length_m': ref_len, 'executed_length_m': act_len,
        'progress_fraction': s_final / ref_len if ref_len else None,
        'goal_reached': reached is not None,
        'time_to_goal_s': (reached - rows[0][0]) if reached is not None else None,
        'final_distance_to_goal_m': end_dist[-1],
        'goal_tolerance_m': goal_tolerance,
        'definition': __doc__.split('\n\n', 1)[1].strip()}


def plot(desired, odom, start, end, result, title, dest):
    """Desired path vs executed trajectory, for checking by eye."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    ref = np.asarray(desired, dtype=float)
    traj = np.array([(o[1], o[2]) for o in odom
                     if (start is None or o[0] >= start) and (end is None or o[0] <= end)]).reshape(-1, 2)
    fig, ax = plt.subplots(figsize=(10, 4.5))
    ax.plot(ref[:, 0], ref[:, 1], 'k--', lw=2, label='desired path')
    if len(traj):
        ax.plot(traj[:, 0], traj[:, 1], color='tab:red', lw=2, label='executed (odom)')
        ax.plot(*traj[0], 'go', label='start'); ax.plot(*traj[-1], 'rs', label='end')
    if result.get('available'):
        ax.set_title(f"{title}\ncross-track mean {result['cross_track_mean_m']:.3f} m, p95 "
                     f"{result['cross_track_p95_m']:.3f} m, max {result['cross_track_max_m']:.3f} m | "
                     f"ADE {result['ade_m']:.3f} m, FDE {result['fde_m']:.3f} m | "
                     f"goal {'reached' if result['goal_reached'] else 'NOT reached'}", fontsize=9)
    ax.set_aspect('equal', adjustable='datalim'); ax.grid(alpha=0.3); ax.legend(loc='best', fontsize=8)
    ys = np.concatenate([ref[:, 1], traj[:, 1]]) if len(traj) else ref[:, 1]
    mid = (ys.min() + ys.max()) / 2
    ax.set_ylim(mid - max(1.0, (ys.max() - ys.min()) / 2 + 0.3), mid + max(1.0, (ys.max() - ys.min()) / 2 + 0.3))
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    fig.tight_layout(); fig.savefig(dest, dpi=110); plt.close(fig)
