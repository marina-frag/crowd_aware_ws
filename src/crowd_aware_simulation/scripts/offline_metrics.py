"""Offline DWAL metrics computed from recorded topics (no ground truth needed).

Called by evaluation.py; every result carries its own definition. Groups:
  clustering   - collision-target labels, mixed / over-split clusters, flicker,
                 stability under noise (ARI), cluster-ID consistency of the
                 recorded clusters and an offline replay of both correspondence
                 modes (upstream / tracking) on the same frames
  intent       - SCAND only: did the controller pick what the demonstrator did
                 (hit rate, Brier, log-loss, angle error, ADE/FDE, baselines)
  latency      - message age through the pipeline, an estimate for the reaction time T
  command_chain- where the commanded speed is reduced along the command pipeline

Angles phi are the polar angle, in the robot frame at pose0, of the point where a
path crosses the circle of radius R (phi = asin(R*k/2) on the arc part).
"""
import bisect
import math
import random


# ---------------------------------------------------------------- geometry
def curv2phi(k, R):
    if abs(k) < math.sqrt(2) / R:
        return math.asin(0.5 * R * k)
    return math.copysign(math.acos(1.0 / (abs(k) * R)), k)


def chord(R, dphi):
    return 2.0 * R * math.sin(0.5 * abs(dphi))


def wrap(a):
    return math.remainder(a, 2.0 * math.pi)


def arc_line_point(k, s):
    """Point at arc length s of an arc-line path from the origin, heading +x
    (arc until 90 degrees, then the tangent)."""
    if abs(k) < 1e-9:
        return s, 0.0
    s90 = math.pi / 2 / abs(k)
    r = 1.0 / k
    if s <= s90:
        th = k * s
        return r * math.sin(th), r * (1 - math.cos(th))
    # 90-degree point (1/|k|, 1/k), then straight on along +-y
    return abs(r), r + math.copysign(s - s90, k)


def interval_iou(a, b):
    inter = max(0.0, min(a[1], b[1]) - max(a[0], b[0]))
    hull = max(a[1], b[1]) - min(a[0], b[0])
    if hull < 1e-12:
        return 1.0 if abs(a[0] - b[0]) < 1e-12 else 0.0
    return inter / hull


def overlap(a, b):
    return max(0.0, min(a[1], b[1]) - max(a[0], b[0]))


def percentile(values, q):
    if not values:
        return None
    v = sorted(values)
    i = (len(v) - 1) * q
    lo, hi = math.floor(i), math.ceil(i)
    return v[lo] + (v[hi] - v[lo]) * (i - lo)


def mean(values):
    return sum(values) / len(values) if values else None


# ------------------------------------------------- clustering (Python replicas)
def cluster_fixed(paths, R, separation, min_span):
    """Upstream fixed-level clustering, including the 3..5-gap behaviour (B1).
    paths: list of dicts with curvature, collision_r. Returns lists of indices."""
    free = [k for k, p in enumerate(paths) if p['collision_r'] > R]
    if not free:
        return []
    out, cl = [], [free[0]]
    phi0 = curv2phi(paths[free[0]]['curvature'], R)
    for k in free[1:]:
        d = k - cl[-1]
        if d > separation:
            if chord(R, curv2phi(paths[cl[-1]]['curvature'], R) - phi0) >= min_span:
                out.append(cl)
            cl = [k]
            phi0 = curv2phi(paths[k]['curvature'], R)
        elif d <= 2:
            cl.append(k)
    if chord(R, curv2phi(paths[free[-1]]['curvature'], R) - phi0) >= min_span:
        out.append(cl)
    return out


def cluster_variable(paths, R, separation, min_span, jump, required, margin, lengths=None):
    """Variable-length (dynamic) clustering of the patch: braking filter with the
    90-degree arc cap, index gap or length jump splits, minimum chord at R."""
    L = lengths if lengths is not None else [p['length'] for p in paths]

    def usable(k):
        c = abs(paths[k]['curvature'])
        return min(L[k], margin + math.pi / 2 / c) if c > 1e-9 else L[k]
    if required is None or not math.isfinite(required):
        return []
    free = [k for k in range(len(paths)) if math.isfinite(L[k]) and usable(k) >= required]
    out, cl = [], []

    def finish():
        if cl and chord(R, curv2phi(paths[cl[-1]]['curvature'], R) -
                        curv2phi(paths[cl[0]]['curvature'], R)) >= min_span:
            out.append(list(cl))
        cl.clear()
    for k in free:
        if cl and (k - cl[-1] > separation or abs(L[k] - L[cl[-1]]) > jump):
            finish()
        cl.append(k)
    finish()
    return out


def replicate(paths, settings, R_level, required, lengths=None):
    """Python replica of the clustering used by the run (fixed or variable)."""
    if settings['variable']:
        return cluster_variable(paths, settings['width_radius'], settings['separation'], settings['min_span'],
                                settings['jump'], required, settings['margin'], lengths)
    if lengths is not None:  # perturb the free extent by the same amount as the length
        paths = [dict(p, collision_r=p['collision_r'] + lengths[i] - p['length']) for i, p in enumerate(paths)]
    return cluster_fixed(paths, R_level, settings['separation'], settings['min_span'])


# ------------------------------------------------------- correspondence replicas
class UpstreamMatcher:
    """Greedy mean-angle matching of dwal_planner; duplicate_bug reproduces B2."""

    def __init__(self, duplicate_bug=False):
        self.bug, self.prev_ids, self.prev_phi = duplicate_bug, [], []

    def step(self, intervals, yaw):
        m = len(intervals)
        means = [(a + b) / 2 for a, b in intervals]
        if m == 0:
            self.prev_ids, self.prev_phi = [], []
            return []
        if not self.prev_ids:
            self.prev_ids, self.prev_phi = list(range(m)), means
            return list(range(m))
        ids = [100] * m
        D = [[abs(c - p) for c in means] for p in self.prev_phi]
        for _ in range(min(len(self.prev_ids), m)):
            r, c = min(((i, j) for i in range(len(D)) for j in range(m)), key=lambda ij: D[ij[0]][ij[1]])
            ids[c] = self.prev_ids[r]
            for row in D:
                row[c] = 1000
            D[r] = [1000] * m
        if m > len(self.prev_ids):
            prev = list(self.prev_ids)
            found = 0 in prev
            for n, cid in enumerate(ids):
                if self.bug:
                    j = 0
                else:
                    j, found = 0, 0 in prev
                if cid == 100:
                    while found:
                        j += 1
                        found = j in prev
                    ids[n] = j
                    prev.append(j)
        self.prev_ids, self.prev_phi = ids, means
        return ids


class TrackingMatcher:
    """Correspondence 'tracking' mode (steps 1-6), same as the C++ patch."""

    def __init__(self, gate=0.2, keep=2):
        self.gate, self.keep, self.tracks, self.next_id, self.prev_yaw = gate, keep, [], 0, None

    def step(self, intervals, yaw):
        if self.prev_yaw is not None and yaw is not None:
            d = wrap(yaw - self.prev_yaw)
            for t in self.tracks:
                t['lo'] -= d
                t['hi'] -= d
        self.prev_yaw = yaw
        self.tracks.sort(key=lambda t: t['lo'] + t['hi'])
        n, m = len(self.tracks), len(intervals)
        iou = [[interval_iou((t['lo'], t['hi']), c) for c in intervals] for t in self.tracks]
        dp = [[0.0] * (m + 1) for _ in range(n + 1)]
        for i in range(1, n + 1):
            for j in range(1, m + 1):
                best = max(dp[i - 1][j], dp[i][j - 1])
                if iou[i - 1][j - 1] >= self.gate:
                    best = max(best, dp[i - 1][j - 1] + iou[i - 1][j - 1])
                dp[i][j] = best
        match = [-1] * m
        i, j = n, m
        while i > 0 and j > 0:
            if iou[i - 1][j - 1] >= self.gate and abs(dp[i][j] - dp[i - 1][j - 1] - iou[i - 1][j - 1]) < 1e-12:
                match[j - 1] = i - 1
                i, j = i - 1, j - 1
            elif abs(dp[i][j] - dp[i - 1][j]) < 1e-12:
                i -= 1
            else:
                j -= 1
        ids, used, born = [], set(), []
        for j, c in enumerate(intervals):
            if match[j] >= 0:
                t = self.tracks[match[j]]
                used.add(match[j])
                t.update(lo=c[0], hi=c[1], missed=0)
                ids.append(t['id'])
            else:
                ids.append(self.next_id)
                born.append({'id': self.next_id, 'lo': c[0], 'hi': c[1], 'missed': 0})
                self.next_id += 1
        kept = []
        for i, t in enumerate(self.tracks):
            if i in used:
                kept.append(t)
            else:
                t['missed'] += 1
                if t['missed'] <= self.keep:
                    kept.append(t)
        self.tracks = kept + born
        return ids


def id_metrics(frames, duration_s):
    """Consistency checks of cluster IDs (no ground truth; physical consistency).
    frames: list of (time, yaw, [(id, (lo, hi)), ...]) ordered by time."""
    switches = crossings = unsupported = duplicates_frames = fragments = 0
    lost = []  # (time, interval, id) of passages that vanished
    prev = None
    for t, yaw, clusters in frames:
        ids = [c[0] for c in clusters]
        if len(ids) != len(set(ids)):
            duplicates_frames += 1
        if prev is not None:
            pt, pyaw, pclusters = prev
            d = wrap(yaw - pyaw) if yaw is not None and pyaw is not None else 0.0
            shifted = [(pid, (a - d, b - d)) for pid, (a, b) in pclusters]
            successors = set()
            for pid, pint in shifted:
                best = max(((interval_iou(pint, cint), cid, cint) for cid, cint in clusters),
                           default=(0.0, None, None))
                if best[0] >= 0.5:
                    successors.add(pid)
                    if best[1] != pid:
                        switches += 1
                same = [cint for cid, cint in clusters if cid == pid]
                if same and interval_iou(pint, same[0]) < 0.1:
                    unsupported += 1
            common = [(pid, pint) for pid, pint in shifted if pid in ids]
            for (ia, a), (ib, b) in zip(common, common[1:]):
                ca = next(c for cid, c in clusters if cid == ia)
                cb = next(c for cid, c in clusters if cid == ib)
                if (a[0] + a[1] - b[0] - b[1]) * (ca[0] + ca[1] - cb[0] - cb[1]) < 0:
                    crossings += 1
            for pid, pint in shifted:
                if pid not in successors:
                    lost.append((t, pint, pid))
            lost = [x for x in lost if t - x[0] <= 0.5]
            for cid, cint in clusters:
                for x in list(lost):
                    if x[0] < t and interval_iou(x[1], cint) >= 0.5:
                        if cid != x[2]:
                            fragments += 1
                        lost.remove(x)
                        break
        prev = (t, yaw, clusters)
    minutes = duration_s / 60.0 if duration_s else None
    rate = (lambda v: v / minutes) if minutes else (lambda v: None)
    return {'frames': len(frames), 'id_switches_per_min': rate(switches),
            'fragmentations_per_min': rate(fragments), 'crossings_per_min': rate(crossings),
            'duplicate_id_frames': duplicates_frames, 'unsupported_inheritance_per_min': rate(unsupported)}


def adjusted_rand_index(a, b):
    """ARI of two labelings of the same items (Hubert & Arabie 1985)."""
    from collections import Counter
    n = len(a)
    if n < 2:
        return None
    comb = lambda x: x * (x - 1) / 2
    pairs = Counter(zip(a, b))
    sa, sb = Counter(a), Counter(b)
    index = sum(comb(v) for v in pairs.values())
    ea, eb = sum(comb(v) for v in sa.values()), sum(comb(v) for v in sb.values())
    expected = ea * eb / comb(n)
    maximum = (ea + eb) / 2
    return 1.0 if maximum == expected else (index - expected) / (maximum - expected)


# ----------------------------------------------------- collision-target labels
def lethal_components(grid):
    """8-connected components of lethal cells (occupancy 100) of an OccupancyGrid dict."""
    import numpy as np
    w, h = grid['width'], grid['height']
    data = np.asarray(grid['data'], dtype=np.int16).reshape(h, w)
    lethal = data >= 100
    labels = np.zeros((h, w), dtype=np.int32)
    current = 0
    for y0, x0 in zip(*np.nonzero(lethal)):
        if labels[y0, x0]:
            continue
        current += 1
        stack = [(y0, x0)]
        labels[y0, x0] = current
        while stack:
            y, x = stack.pop()
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    yy, xx = y + dy, x + dx
                    if 0 <= yy < h and 0 <= xx < w and lethal[yy, xx] and not labels[yy, xx]:
                        labels[yy, xx] = current
                        stack.append((yy, xx))
    ys, xs = np.nonzero(labels)
    cx = grid['origin'][0] + (xs + 0.5) * grid['resolution']
    cy = grid['origin'][1] + (ys + 0.5) * grid['resolution']
    return np.stack([cx, cy], axis=1), labels[ys, xs]


def target_label(end, points, components, footprint, reach):
    """Lethal component nearest to the footprint at the path's last free pose
    (the obstacle the path runs into), or 'free' if the path reached the horizon."""
    import numpy as np
    if reach or len(points) == 0:
        return 'free'
    x, y, th = end
    c, s = math.cos(th), math.sin(th)
    local = (points - [x, y]) @ np.array([[c, -s], [s, c]])
    xmin, xmax, ymin, ymax = footprint
    dx = np.maximum(np.maximum(xmin - local[:, 0], 0), local[:, 0] - xmax)
    dy = np.maximum(np.maximum(ymin - local[:, 1], 0), local[:, 1] - ymax)
    d = np.hypot(dx, dy)
    i = int(np.argmin(d))
    return int(components[i]) if d[i] < 0.5 else 'unknown'


# ----------------------------------------------------------------- metrics
def clustering_metrics(groups, settings, costmaps, footprint, duration_s, seed=0):
    """groups: {name: [frame, ...]}, frame = dict(time, yaw, R_level, paths, clusters,
    required). paths: dicts curvature, length, collision_r, end (x, y, th), reach.
    clusters: recorded [(id, [member indices])]."""
    rng = random.Random(seed)
    cm_times = [c['time'] for c in costmaps]
    comp_cache = {}
    out = {}
    R = settings['reference_radius']
    for name, frames in groups.items():
        if not frames:
            continue
        mixed = clusters_total = runs = oversplit = 0
        counts, ari_values, agree = [], [], []
        recorded_ids = []
        replay = {'upstream': [], 'tracking': []}
        matchers = {'upstream': UpstreamMatcher(duplicate_bug=not settings['variable']),
                    'tracking': TrackingMatcher(settings['gate'], settings['keep'])}
        for f in frames:
            paths, clusters = f['paths'], f['clusters']
            counts.append((f['time'], len(clusters)))
            intervals = []
            for cid, members in clusters:
                a = curv2phi(paths[members[0]]['curvature'], R)
                b = curv2phi(paths[members[-1]]['curvature'], R)
                intervals.append((min(a, b), max(a, b)))
            recorded_ids.append((f['time'], f['yaw'], list(zip([c[0] for c in clusters], intervals))))
            for mode, matcher in matchers.items():
                replay[mode].append((f['time'], f['yaw'], list(zip(matcher.step(intervals, f['yaw']), intervals))))
            # replica check and stability (ARI) under length noise sigma
            rerun = lambda L=None: replicate(paths, settings, f['R_level'], f['required'], L)
            clean = rerun()
            agree.append(f['replica_match'])
            noisy_L = [p['length'] + rng.gauss(0.0, settings['noise_sigma']) for p in paths]
            noisy = rerun(noisy_L)
            lab = lambda cl: [next((n for n, members in enumerate(cl) if k in members), -1) for k in range(len(paths))]
            if clean or noisy:
                ari = adjusted_rand_index(lab(clean), lab(noisy))
                if ari is not None:
                    ari_values.append(ari)
            # collision-target labels (needs a costmap no older than 0.5 s)
            i = bisect.bisect_right(cm_times, f['time']) - 1
            if i < 0 or f['time'] - cm_times[i] > 0.5 or not clusters:
                continue
            if i not in comp_cache:
                comp_cache = {i: lethal_components(costmaps[i])}
            points, comps = comp_cache[i]
            label = {}
            for _, members in clusters:
                for k in members:
                    p = paths[k]
                    label[k] = target_label(p['end'], points, comps, footprint, p['reach'])
            for _, members in clusters:
                clusters_total += 1
                if len({label[k] for k in members}) >= 2:
                    mixed += 1
            ordered = sorted((k, n) for n, (_, members) in enumerate(clusters) for k in members)
            run_clusters, run_label = set(), None
            for k, n in ordered + [(None, None)]:
                lab_k = label.get(k) if k is not None else object()
                if run_label is not None and lab_k != run_label:
                    runs += 1
                    oversplit += len(run_clusters) >= 2
                    run_clusters = set()
                if k is not None:
                    run_label = lab_k
                    run_clusters.add(n)
        # flicker: a change of the cluster count undone within 0.5 s
        changes = flicker = 0
        for idx in range(1, len(counts)):
            if counts[idx][1] != counts[idx - 1][1]:
                changes += 1
                for later in counts[idx + 1:]:
                    if later[0] - counts[idx][0] > 0.5:
                        break
                    if later[1] == counts[idx - 1][1]:
                        flicker += 1
                        break
        minutes = duration_s / 60.0 if duration_s else None
        out[name] = {
            'frames': len(frames),
            'replica_agreement_fraction': mean([float(a) for a in agree]),
            'mixed_cluster_fraction': mixed / clusters_total if clusters_total else None,
            'oversplit_run_fraction': oversplit / runs if runs else None,
            'labelled_clusters': clusters_total,
            'count_changes_per_min': changes / minutes if minutes else None,
            'flicker_per_min': flicker / minutes if minutes else None,
            'stability_ari_mean': mean(ari_values),
            'stability_ari_p05': percentile(ari_values, 0.05),
            'ids_recorded': id_metrics(recorded_ids, duration_s),
            'ids_replay_upstream': id_metrics(replay['upstream'], duration_s),
            'ids_replay_tracking': id_metrics(replay['tracking'], duration_s)}
    out['definition'] = (
        'Per clustering frame. Collision-target label of a clustered path = lethal costmap component '
        'nearest to the footprint at its last free pose (or "free" if it reaches the horizon); '
        'mixed = clusters whose members have >= 2 labels; over-split = runs of consecutive clustered '
        'paths with one label that span >= 2 clusters. Flicker = cluster-count change undone within 0.5 s. '
        f'Stability = ARI between the clustering of the recorded paths and of the same paths with free '
        f'length noise sigma={settings["noise_sigma"]} m (Python replica; replica_agreement_fraction = '
        'frames where the replica reproduces the recorded clusters from a sampled_paths message '
        'received from 0.25 s before to 0.15 s after them; misses are mostly source messages '
        'absent from the bag). IDs: switch = consecutive-frame '
        'clusters with interval IoU >= 0.5 but a different ID; fragmentation = a passage lost and found '
        'again within 0.5 s under another ID; crossing = two IDs swap their angular order; unsupported '
        'inheritance = same ID with IoU < 0.1; intervals at the reference radius, previous frame rotated by '
        'the heading change. ids_recorded = IDs as published; ids_replay_* = both correspondence modes '
        'replayed offline on the same cluster intervals.')
    return out


def intent_metrics(diags, odom, reference, scans_to_points, footprint, R_default, false_stop):
    """SCAND: the demonstrator's future path is the ground truth of intent.
    diags: [(t, dict)] intent diagnostics; odom: [(t, x, y, yaw)] recorded pose;
    reference: [(t, v, w)]; scans_to_points(t) -> Nx2 obstacle points in odom or None."""
    import numpy as np
    ot = [o[0] for o in odom]
    rt = [r[0] for r in reference]
    rows = []
    for t, d in diags:
        pose0 = d.get('pose0') or []
        if len(pose0) < 3 or not d.get('clusters'):
            continue
        R = d.get('level') or R_default
        x0, y0, th0 = pose0
        i = bisect.bisect_left(ot, t)
        future, s = [], 0.0
        last = (x0, y0)
        cross = None
        for o in odom[i:]:
            s += math.dist(last, o[1:3])
            last = o[1:3]
            future.append((s, o[1], o[2]))
            if math.hypot(o[1] - x0, o[2] - y0) >= R:
                cross = o
                break
        if cross is None:
            continue
        phi_true = wrap(math.atan2(cross[2] - y0, cross[1] - x0) - th0)
        clusters = d['clusters']
        inside = [n for n, c in enumerate(clusters) if c['phi_min'] - 1e-9 <= phi_true <= c['phi_max'] + 1e-9]
        dist = [0.0 if n in inside else min(abs(phi_true - c['phi_min']), abs(phi_true - c['phi_max']))
                for n, c in enumerate(clusters)]
        correct = int(np.argmin(dist))
        belief = [c.get('belief', 0.0) for c in clusters]
        total = sum(belief)
        belief = [b / total for b in belief] if total > 0 else [1.0 / len(clusters)] * len(clusters)
        brier = sum((b - (n == correct)) ** 2 for n, b in enumerate(belief))
        uniform_brier = sum((1.0 / len(clusters) - (n == correct)) ** 2 for n in range(len(clusters)))
        j = bisect.bisect_right(rt, t) - 1
        k_ref = None
        if j >= 0 and abs(reference[j][1]) > 0.02:
            k_ref = reference[j][2] / reference[j][1]
        user_cluster = next((n for n, c in enumerate(clusters)
                             if c['phi_min'] <= d['phi_user'] <= c['phi_max']), None)
        # ADE/FDE at equal arc length, robot frame at pose0
        c0, s0 = math.cos(th0), math.sin(th0)
        true_local = [(sv, c0 * (x - x0) + s0 * (y - y0), -s0 * (x - x0) + c0 * (y - y0)) for sv, x, y in future]

        def ade_fde(k):
            if k is None or not true_local:
                return None, None
            errs = [math.dist(arc_line_point(k, sv), (xl, yl)) for sv, xl, yl in true_local]
            return sum(errs) / len(errs), errs[-1]
        ade, fde = ade_fde(d['k_star'])
        b_ade, b_fde = ade_fde(k_ref)
        exception = None
        points = scans_to_points(t)
        if points is not None and len(points):
            xmin, xmax, ymin, ymax = footprint
            exception = False
            for idx in range(len(future) - 1):
                _, x, y = future[idx]
                _, xn, yn = future[idx + 1]
                th = math.atan2(yn - y, xn - x)
                c, s = math.cos(th), math.sin(th)
                local = (points - [x, y]) @ np.array([[c, -s], [s, c]])
                if np.any((local[:, 0] >= xmin) & (local[:, 0] <= xmax) &
                          (local[:, 1] >= ymin) & (local[:, 1] <= ymax)):
                    exception = True
                    break
            near = float(np.min(np.hypot(points[:, 0] - x0, points[:, 1] - y0)))
        else:
            near = None
        rows.append(dict(
            hit=d['map'] in inside if d['map'] >= 0 else False, inside=bool(inside), brier=brier,
            uniform_brier=uniform_brier, logloss=-math.log(max(belief[correct], 1e-6)),
            user_cluster_hit=user_cluster in inside if user_cluster is not None else False,
            phi_err=abs(wrap(d['phi_star'] - phi_true)), phi_user_err=abs(wrap(d['phi_user'] - phi_true)),
            phi_ref_err=abs(wrap(curv2phi(k_ref, R) - phi_true)) if k_ref is not None else None,
            ade=ade, fde=fde, b_ade=b_ade, b_fde=b_fde, exception=exception, near=near))

    def summary(sel):
        if not sel:
            return None
        get = lambda key: [r[key] for r in sel if r[key] is not None]
        return {'frames': len(sel), 'hit_rate': mean([float(r['hit']) for r in sel]),
                'baseline_user_cluster_hit_rate': mean([float(r['user_cluster_hit']) for r in sel]),
                'phi_true_inside_any_cluster_fraction': mean([float(r['inside']) for r in sel]),
                'brier_mean': mean(get('brier')), 'baseline_uniform_brier_mean': mean(get('uniform_brier')),
                'logloss_mean': mean(get('logloss')),
                'phi_error_mean_rad': mean(get('phi_err')),
                'baseline_phi_user_error_mean_rad': mean(get('phi_user_err')),
                'baseline_reference_curvature_phi_error_mean_rad': mean(get('phi_ref_err')),
                'ade_mean_m': mean(get('ade')), 'fde_mean_m': mean(get('fde')),
                'baseline_ade_mean_m': mean(get('b_ade')), 'baseline_fde_mean_m': mean(get('b_fde'))}
    feasible = [r for r in rows if r['exception'] is not True]
    bins = {}
    for lo, hi in ((0.0, 1.0), (1.0, 2.0), (2.0, math.inf)):
        bins[f'obstacle_{lo:g}_{hi:g}m'] = summary([r for r in feasible if r['near'] is not None and lo <= r['near'] < hi])
    return {
        'all_frames': summary(rows),
        'iwalk_feasible_frames': summary(feasible),
        'iwalk_exception_fraction': mean([float(r['exception']) for r in rows if r['exception'] is not None]),
        'by_obstacle_distance': bins,
        'false_stop': false_stop,
        'definition': (
            'Per intent-diagnostics message. phi_true = polar angle (robot frame at pose0) where the recorded '
            'future path first leaves the circle of the level radius. hit = phi_true inside the MAP cluster; '
            'Brier = sum_i (p_i - y_i)^2 with y = cluster containing (or nearest to) phi_true; log-loss = '
            '-log p_correct. phi_error = |phi_star - phi_true|. ADE/FDE: chosen arc-line path vs the true '
            'path at equal arc length up to the circle. Baselines: cluster containing phi_user, uniform belief, '
            'arc of the reference curvature w/v ("user only"). iWalk exception: the iWalk footprint along the '
            'true path overlaps a scan point (path not feasible for iWalk), reported separately; by_obstacle_'
            'distance uses the nearest scan point to pose0. false_stop = share of grid time with selected '
            'v < 0.01 m/s while the recorded robot moved faster than 0.1 m/s.')}


def latency(receipts, stamps):
    """Age of information through the pipeline from bag receipt times.
    receipts: {stage: [t...]} sorted; stamps: {'costmap': [(receipt, header_stamp)]}."""
    def gap(after, before, limit):
        # Gaps above limit are not delays: the upstream node stopped publishing
        # (e.g. the collision monitor while stopped) and the stage times out instead.
        b = receipts.get(before) or []
        out = []
        for t in receipts.get(after) or []:
            i = bisect.bisect_right(b, t) - 1
            if i >= 0 and t - b[i] <= limit:
                out.append(t - b[i])
        return out
    stages = {}
    scans = receipts.get('scan') or []
    a1 = []
    for receipt, stamp in stamps.get('costmap', []):
        i = bisect.bisect_right(scans, stamp) - 1
        if i >= 0:
            a1.append(receipt - scans[i])
    stages['scan_to_costmap'] = a1
    for name, (after, before) in {'costmap_to_paths': ('sampled', 'costmap'),
                                  'paths_to_clusters': ('clusters', 'sampled'),
                                  'clusters_to_selected_cmd': ('selected', 'clusters'),
                                  'selected_to_smoothed': ('smoothed', 'selected'),
                                  'smoothed_to_safe': ('safe', 'smoothed'),
                                  'safe_to_final': ('final', 'safe')}.items():
        stages[name] = gap(after, before, 0.3 if name == 'safe_to_final' else 1.0)
    table = {k: {'mean_s': mean(v), 'p95_s': percentile(v, 0.95), 'samples': len(v)} for k, v in stages.items() if v}
    present = [v for v in table.values()]
    return {'stages': table,
            'total_mean_s': sum(v['mean_s'] for v in present) if present else None,
            'total_p95_sum_s': sum(v['p95_s'] for v in present) if present else None,
            'definition': 'Per stage, age of the newest input at each output message (bag receipt times; '
                          'scan_to_costmap uses the costmap header stamp). Sum of means ~ mean reaction delay, '
                          'sum of p95 ~ conservative bound (for the reaction time T of the braking model). '
                          'Gaps above 1 s (0.3 s for the watchdog input) are dropped: the upstream node stopped '
                          'publishing. Resolution = recorder clock step (0.1 s with Gazebo /clock). '
                          'Excludes sensor acquisition and actuator delays.'}


def command_chain(series, grid, hold):
    """Mean speed per command topic and mean drop per stage on a common grid."""
    def at(rows, t):
        ts = [r[0] for r in rows]
        i = bisect.bisect_right(ts, t) - 1
        return rows[i][1] if i >= 0 and t - rows[i][0] <= hold else None
    order = [k for k in ('reference', 'selected', 'smoothed', 'safe', 'final') if series.get(k)]
    values = {k: [at(series[k], t) for t in grid] for k in order}
    means = {k: mean([v for v in vals if v is not None]) for k, vals in values.items()}
    drops = {}
    for a, b in zip(order, order[1:]):
        d = [x - y for x, y in zip(values[a], values[b]) if x is not None and y is not None]
        drops[f'{a}_to_{b}'] = {'mean_drop_mps': mean(d),
                                'share_drop_over_0p02': mean([float(x > 0.02) for x in d])}
    return {'mean_speed_mps': means, 'drops': drops,
            'definition': 'Linear speed of each command topic on the evaluation grid (latest message held '
                          f'up to {hold} s); drop = upstream minus downstream speed at the same time. The '
                          'collision monitor does not republish zero commands, so its mean covers only the '
                          'times it publishes.'}


# ------------------------------------------------------------- bag collector
def _params(bag_dir, names, node_key):
    """ros__parameters of node_key from the first parameter file found in the run."""
    import yaml
    for name in names:
        for path in (bag_dir / name, bag_dir / 'run' / name):
            try:
                data = yaml.safe_load(path.read_text())
            except (OSError, PermissionError):
                continue
            for key, value in (data or {}).items():
                if key.rstrip('/').endswith(node_key) and 'ros__parameters' in value:
                    return value['ros__parameters']
    return None


class Collector:
    """Gathers the raw topics evaluation.py does not keep, then computes the groups."""

    def __init__(self, cfg, bag_dir):
        t = cfg['topics']
        self.cfg, self.t = cfg, t
        self.dwal = _params(bag_dir, ('dwal.yaml', 'dwal_planner_dwal_clustering_parameters.yaml'), 'dwal_clustering') or {}
        self.shared = _params(bag_dir, ('shared_control_off.yaml', 'shared_control_on.yaml',
                                        'shared_controller_parameters.yaml'), 'shared_controller') or {}
        self.sampled, self.clusters, self.required, self.costmaps, self.diags = [], {}, {}, [], []
        self.chain = {k: [] for k in ('reference', 'selected', 'smoothed', 'safe', 'final')}
        self.receipts = {k: [] for k in ('scan', 'costmap', 'sampled', 'clusters', 'selected', 'smoothed', 'safe', 'final')}
        self.stamps = {'costmap': []}
        self.chain_topics = {t.get('reference_cmd'): 'reference', t.get('selected_cmd'): 'selected',
                             t.get('smoothed_cmd'): 'smoothed', t.get('collision_checked_cmd'): 'safe',
                             t.get('final_cmd'): 'final'}
        self.cluster_topics = {t.get('clusters_far'): 'far', t.get('clusters_near'): 'near'}
        self.diag_topics = {t.get('far_diagnostics'): 'far', t.get('near_diagnostics'): 'near'}

    def topics(self):
        t = self.t
        keys = ('costmap', 'intent_diagnostics', 'smoothed_cmd', 'collision_checked_cmd', 'final_cmd')
        return {t[k] for k in keys if t.get(k)}

    def event(self, now, topic, msg):
        import json as _json
        import numpy as np
        if topic in self.chain_topics and topic is not None:
            v = msg.twist.linear.x if hasattr(msg, 'twist') else msg.linear.x
            if math.isfinite(v):
                self.chain[self.chain_topics[topic]].append((now, v))
            name = self.chain_topics[topic]
            if name in self.receipts:
                self.receipts[name].append(now)
        if topic == self.t.get('scan'):
            self.receipts['scan'].append(now)
        elif topic == self.t.get('costmap'):
            info = msg.info
            if abs(info.origin.orientation.z) < 1e-6:
                self.costmaps.append({'time': now, 'width': info.width, 'height': info.height,
                                      'resolution': info.resolution,
                                      'origin': (info.origin.position.x, info.origin.position.y),
                                      'data': np.asarray(msg.data, dtype=np.int8)})
            self.receipts['costmap'].append(now)
            self.stamps['costmap'].append((now, msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9))
        elif topic == self.t.get('sampled_paths'):
            self.receipts['sampled'].append(now)
            if len(msg.pose0) < 3:
                return
            x0, y0 = msg.pose0[0], msg.pose0[1]
            paths = []
            for p in msg.paths:
                poses = p.poses
                length = sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(poses, poses[1:]))
                end = (poses[-1].x, poses[-1].y, poses[-1].theta) if poses else (x0, y0, msg.pose0[2])
                paths.append({'curvature': float(p.curvature), 'length': length,
                              'collision_r': float(p.colission_r), 'end': end,
                              'end_radius': math.hypot(end[0] - x0, end[1] - y0)})
            self.sampled.append((now, float(msg.pose0[2]), paths))
        elif topic in self.cluster_topics and topic is not None:
            self.receipts['clusters'].append(now)
            self.clusters.setdefault(self.cluster_topics[topic], []).append(
                (now, [(int(c.id), [float(p.curvature) for p in c.paths], [int(p.cost) for p in c.paths])
                       for c in msg.clusters]))
        elif topic in self.diag_topics and topic is not None:
            try:
                req = _json.loads(msg.data).get('required_length')
            except (ValueError, TypeError):
                return
            self.required.setdefault(self.diag_topics[topic], []).append((now, req))
        elif topic == self.t.get('intent_diagnostics'):
            try:
                self.diags.append((now, _json.loads(msg.data)))
            except ValueError:
                pass

    def footprint(self):
        fp = self.shared.get('footprint') or []
        if len(fp) >= 6:
            xs, ys = fp[0::2], fp[1::2]
            return min(xs), max(xs), min(ys), max(ys)
        return -0.103, 0.865, -0.327, 0.327

    def results(self, start, duration, grid, extra):
        out = {}
        end = start + duration
        variable = bool(self.dwal.get('dwal_clustering/variable_length', False))
        levels = self.dwal.get('common/levels') or [self.cfg.get('reference_radius_m', 1.8)]
        postfix = self.dwal.get('dwal_clustering/postfix') or (['far'] if variable else ['near', 'far'])
        level_of = dict(zip(postfix, levels))
        settings = {'variable': variable, 'reference_radius': self.cfg.get('reference_radius_m', 1.8),
                    'width_radius': levels[-1], 'separation': int(self.dwal.get('dwal_clustering/cluster_separation', 5)),
                    'min_span': float(self.dwal.get('dwal_clustering/min_cluster_span', 0.2)),
                    'jump': float(self.dwal.get('dwal_clustering/length_jump_threshold', 0.3)),
                    'margin': float(self.shared.get('stop_margin', 0.05)),
                    'gate': float(self.dwal.get('dwal_clustering/correspondence_gate', 0.2)),
                    'keep': int(self.dwal.get('dwal_clustering/track_keep_frames', 2)),
                    'noise_sigma': 0.02}
        st = [s[0] for s in self.sampled]
        groups = {}
        for name, msgs in self.clusters.items():
            R_level = level_of.get(name, levels[-1])
            req_rows = self.required.get(name, [])
            rt = [r[0] for r in req_rows]
            frames = []
            for now, clusters in msgs:
                if not start <= now < end:
                    continue
                j = bisect.bisect_right(rt, now) - 1
                required = req_rows[j][1] if j >= 0 and now - rt[j] <= 0.2 else None
                # The clusters come from a sampled_paths message received just before or
                # (large message, 0.1 s recorder clock step) just after them. Take the
                # newest whose clustering the replica reproduces, else the newest that
                # contains every member.
                chosen = None
                i = bisect.bisect_right(st, now + 0.15) - 1
                while i >= 0 and now - st[i] <= 0.25:
                    _, yaw, paths = self.sampled[i]
                    index = {p['curvature']: k for k, p in enumerate(paths)}
                    members = [(cid, [index.get(c) for c in curvatures]) for cid, curvatures, _ in clusters]
                    if all(ks and None not in ks for _, ks in members):
                        match = (sorted(map(tuple, replicate(paths, settings, R_level, required))) ==
                                 sorted(tuple(ks) for _, ks in members))
                        if chosen is None or (match and not chosen[3]):
                            chosen = (yaw, paths, members, match)
                        if match:
                            break
                    i -= 1
                if chosen is None:
                    continue
                yaw, paths, members, match = chosen
                for p in paths:
                    p['reach'] = (p['end_radius'] >= R_level - 0.05) if variable else (p['collision_r'] >= R_level)
                frames.append({'time': now, 'yaw': yaw, 'R_level': R_level, 'paths': paths,
                               'clusters': members, 'required': required, 'replica_match': match})
            groups[name] = frames
        if groups:
            out['clustering'] = clustering_metrics(groups, settings, self.costmaps, self.footprint(), duration)
        out['settings'] = {'correspondence': self.dwal.get('dwal_clustering/correspondence', 'upstream'),
                           'variable_length': variable, 'replay': settings}
        out['latency'] = latency(self.receipts, self.stamps)
        if len([k for k, v in self.chain.items() if v]) >= 2:
            out['command_chain'] = command_chain(self.chain, grid, hold=0.15)
        # cost factor max(s_min, 1-c/c_max): applied by fixed DWAL only
        s_min, c_max = float(self.shared.get('s_min', 0.2)), float(self.shared.get('c_max', 255.0))
        # Dynamic DWAL dropped the cost speed factor together with adding the
        # 'correspondence' parameter (2026-11-04); older dynamic runs still applied it.
        applied = not variable or 'dwal_clustering/correspondence' not in self.dwal
        out['cost_factor'] = (self._cost_factor(start, end, s_min, c_max) if applied else
                              {'applied': False, 'definition': 'Dynamic DWAL has no cost speed factor (v from the braking model only).'})
        if str(self.cfg.get('scenario', '')).startswith('scand:') and self.diags:
            diags = [(t, d) for t, d in self.diags if start <= t < end]
            out['intent'] = intent_metrics(diags, extra.odom, extra.reference, extra.points_at,
                                           self.footprint(), settings['reference_radius'],
                                           extra.false_stop(grid))
        return out

    def _cost_factor(self, start, end, s_min, c_max):
        groups = {}
        for name, msgs in self.clusters.items():
            factors = [max(s_min, 1.0 - min(max(c / c_max, 0.0), 1.0))
                       for now, clusters in msgs if start <= now < end
                       for _, _, costs in clusters for c in costs]
            groups[name] = {'paths': len(factors), 'mean_factor': mean(factors),
                            'share_at_s_min': mean([float(f <= s_min + 1e-9) for f in factors])}
        return {'applied': True, 's_min': s_min, 'c_max': c_max, 'by_group': groups,
                'definition': 'Fixed DWAL speed factor max(s_min, 1 - cost/c_max) of every clustered path '
                              '(cost = max footprint cost up to the level radius).'}
