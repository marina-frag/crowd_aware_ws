#!/usr/bin/env python3
"""Readable summary of run metrics (crowd_aware_ws simulation and scand_ws runs).

Runs on the host with plain python3 (no ROS, no build). A run is a directory with
evaluation.json and bag/; its metrics are in analysis_v3/metrics.json, written by
crowd_aware evaluation.py. With --evaluate, missing or outdated metrics are computed
in the iwalk_dwal image using the host's evaluation code (src/.../scripts and config/
are mounted), so changes to evaluation.py or to this script need no build.

Usage:
  # Summary of all runs in crowd_aware_ws/rosbags
  python3 ~/crowd_aware_ws/scripts/metrics_summary.py
  # Also scand_ws runs; compute metrics that are missing/outdated first
  python3 ~/crowd_aware_ws/scripts/metrics_summary.py ~/crowd_aware_ws/rosbags ~/scand_ws/results --evaluate
  # Recompute everything (e.g. after changing evaluation.py) and save the tables
  python3 ~/crowd_aware_ws/scripts/metrics_summary.py --evaluate --force --markdown summary.md --csv summary.csv

Statistics (section "Στατιστική"): runs paired by scenario, seed and semantics; for each
pair of variants the paired differences get a Wilcoxon signed-rank test (exact p),
the matched-pairs rank-biserial correlation (effect size) and a bootstrap 95% CI of
the median difference. SCAND runs are not paired (one recording per run).
"""
import argparse
import csv
import json
import math
import os
import random
import subprocess
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
PACKAGE = REPO/'src'/'crowd_aware_simulation'
CONTAINER_PACKAGE = '/opt/dwal_ws/src/crowd_aware_simulation'
SCHEMA = 6  # evaluation.py metrics.json schema_version this summary expects
CODES = [('clustered', 'cluster'), ('invalid_or_filtered', '-1'), ('braking_rejected', '-2'),
         ('outside_slice', '-3'), ('min_span_rejected', '-4')]


def find_runs(roots):
    """(runs, no_bag): directories with evaluation.json and bag/, and those without bag/
    (e.g. scand_ws results from before runs were recorded)."""
    runs, no_bag = set(), set()
    for root in roots:
        root = root.expanduser().resolve()
        if not root.is_dir(): raise SystemExit(f'not a directory: {root}')
        for meta in root.rglob('evaluation.json'):
            (runs if (meta.parent/'bag').is_dir() else no_bag).add(meta.parent)
    return sorted(runs), sorted(no_bag)


def metrics_path(run):
    return run/'analysis_v3'/'metrics.json'


def load(run):
    try: return json.loads(metrics_path(run).read_text())
    except (OSError, ValueError): return None


def evaluate(run, image):
    """Run evaluation.py in the image as root (run directories are root-owned), then
    give analysis_v3 back to the host user."""
    script = (f'python3 {CONTAINER_PACKAGE}/scripts/evaluation.py evaluate /run >/dev/null && '
              f'chown -R {os.getuid()}:{os.getgid()} /run/analysis_v3')
    cmd = ['docker', 'run', '--rm', '-v', f'{run}:/run',
           '-v', f'{PACKAGE/"scripts"}:{CONTAINER_PACKAGE}/scripts:ro',
           '-v', f'{PACKAGE/"config"}:{CONTAINER_PACKAGE}/config:ro',
           image, 'bash', '/dwal_entrypoint.sh', 'bash', '-c', script]
    print(f'evaluating {run.name} ...', flush=True)
    done = subprocess.run(cmd, capture_output=True, text=True)
    if done.returncode != 0:
        print(f'  FAILED: {(done.stderr or done.stdout).strip().splitlines()[-1:]}')


def num(value, digits=2, scale=1.0):
    return 'n/a' if value is None else f'{value*scale:.{digits}f}'


def pair(a, b, digits=2):
    return 'n/a' if a is None and b is None else f'{num(a, digits)} / {num(b, digits)}'


def with_share(value, fraction, unit, digits=1):
    if value is None: return 'n/a'
    return f'{value:.{digits}f} {unit}' + (f' ({fraction*100:.0f}%)' if fraction is not None else '')


def rows_for(run, data):
    c = data.get('condition', {})
    x = data.get('additional_metrics', {})
    s = x.get('smoothness', {})
    requested = data.get('requested_motion_s')
    standstill = data.get('standstill_while_requested_s')
    label = {
        'run': run.name, 'scenario': c.get('scenario'), 'controller': c.get('controller'),
        'variant': c.get('variant'), 'semantics': c.get('semantics'), 'seed': c.get('seed'),
        'window_s': data.get('duration_observed_s'), 'motion_source': data.get('motion_source', '/odom'),
        'all_metrics_available': x.get('all_metrics_available')}
    values = {
        'collisions': data.get('collision_episodes'),
        'obstacle_collisions': data.get('obstacle_collision_episodes'),
        'human_collisions': data.get('human_collision_episodes'),
        'standstill_s': standstill, 'requested_motion_s': requested,
        'zone_time_s': x.get('personal_zone_time_s'), 'zone_time_fraction': x.get('personal_zone_fraction'),
        'zone_path_m': x.get('personal_zone_path_m'), 'zone_path_fraction': x.get('personal_zone_path_fraction'),
        'path_length_m': x.get('path_length_m'),
        'human_min_m': x.get('minimum_human_center_distance_m'), 'human_mean_m': x.get('mean_human_center_distance_m'),
        'obstacle_min_m': x.get('minimum_obstacle_distance_m'), 'obstacle_mean_m': x.get('mean_obstacle_distance_m')}
    proposed, intervention = x.get('proposed_path') or {}, x.get('intervention') or {}
    for key in ('shared_clearance_min_m', 'shared_clearance_mean_m', 'human_clearance_min_m',
                'human_clearance_mean_m', 'shared_length_mean_m', 'shared_length_min_m'):
        values[f'proposed_{key}'] = proposed.get(key)
    for key in ('curvature_difference_mean_1pm', 'curvature_difference_max_1pm', 'speed_ratio_mean'):
        values[f'intervention_{key}'] = intervention.get(key)
    for quantity, unit in (('linear_acceleration', 'mps2'), ('angular_acceleration', 'radps2'),
                           ('linear_jerk', 'mps3'), ('angular_jerk', 'radps3')):
        for stat in ('mean', 'max', 'min'):
            values[f'{quantity}_{stat}_abs'] = s.get(f'{quantity}_{stat}_abs_{unit}')
    off = data.get('offline') or {}
    chain = off.get('command_chain') or {}
    for key, v in (chain.get('mean_speed_mps') or {}).items(): values[f'chain_{key}_mps'] = v
    for key, v in (chain.get('drops') or {}).items(): values[f'drop_{key}_mps'] = v.get('mean_drop_mps')
    lat = off.get('latency') or {}
    values['latency_mean_s'], values['latency_p95_sum_s'] = lat.get('total_mean_s'), lat.get('total_p95_sum_s')
    cost = off.get('cost_factor') or {}
    for group, v in (cost.get('by_group') or {}).items():
        values[f'cost_{group}_share_s_min'], values[f'cost_{group}_mean'] = v.get('share_at_s_min'), v.get('mean_factor')
    values['cost_applied'] = cost.get('applied')
    for group, v in (off.get('clustering') or {}).items():
        if not isinstance(v, dict): continue
        for key in ('mixed_cluster_fraction', 'oversplit_run_fraction', 'flicker_per_min', 'count_changes_per_min',
                    'stability_ari_mean', 'replica_agreement_fraction'):
            values[f'cl_{group}_{key}'] = v.get(key)
        for src in ('recorded', 'replay_upstream', 'replay_tracking'):
            ids = v.get(f'ids_{src}') or {}
            for key in ('id_switches_per_min', 'fragmentations_per_min', 'crossings_per_min',
                        'duplicate_id_frames', 'unsupported_inheritance_per_min'):
                values[f'id_{group}_{src}_{key}'] = ids.get(key)
    label['correspondence'] = (off.get('settings') or {}).get('correspondence')
    intent = off.get('intent') or {}
    for scope in ('all_frames', 'iwalk_feasible_frames'):
        for key, v in (intent.get(scope) or {}).items(): values[f'intent_{scope}_{key}'] = v
    values['intent_exception_fraction'] = intent.get('iwalk_exception_fraction')
    values['intent_false_stop'] = (intent.get('false_stop') or {}).get('fraction')
    values['safety_not_computed'] = bool(data.get('safety_metrics'))
    diagnostics = x.get('dwal_diagnostics', {})
    for topic, d in sorted(diagnostics.items()):
        group = topic.rsplit('/', 1)[-1].replace('clusters_', '').replace('_diagnostics', '')
        shares = d.get('mean_path_fraction') or {}
        for key, _ in CODES: values[f'dwal_{group}_{key}'] = shares.get(key)
        values[f'dwal_{group}_no_cluster'] = d.get('no_cluster_message_fraction')
        values[f'dwal_{group}_messages'] = d.get('messages')
    return {**label, **values}


def markdown(rows):
    out = []
    def table(title, header, cells):
        out.append(f'\n## {title}\n')
        out.append('| ' + ' | '.join(header) + ' |')
        out.append('|' + '---|'*len(header))
        for cell in cells: out.append('| ' + ' | '.join(str(v) for v in cell) + ' |')
    out.append('# Μετρικές runs\n')
    for i, r in enumerate(rows, 1):
        seed = '' if str(r['scenario']).startswith('scand:') else f'seed {r["seed"]}, '  # no seed for a recording
        corr = ', αντιστοίχιση tracking' if r.get('correspondence') == 'tracking' else ''
        out.append(f'- **R{i}** `{r["run"]}`: {r["scenario"]}, {r["controller"]} ({r["variant"]}{corr}), '
                   f'semantics {r["semantics"]}, {seed}κίνηση από `{r["motion_source"]}`'
                   + ('' if r['all_metrics_available'] else ' (κάποιες μετρικές n/a)'))
    na = lambda r, text: '— ‡' if r.get('safety_not_computed') else text
    table('Ασφάλεια και ανταπόκριση',
          ['Run', 'Παράθυρο', 'Συγκρούσεις (σύνολο)', 'Εμπόδιο / HC', 'Ακινησία ενώ ζητείται κίνηση'],
          [[f'R{i}', num(r['window_s'], 1)+' s', na(r, num(r['collisions'], 0)),
            na(r, pair(r['obstacle_collisions'], r['human_collisions'], 0)),
            with_share(r['standstill_s'], r['standstill_s']/r['requested_motion_s']
                       if r['standstill_s'] is not None and r['requested_motion_s'] else None, 's')]
           for i, r in enumerate(rows, 1)])
    scand = lambda r: str(r['scenario']).startswith('scand:')
    mark = lambda r: ' †' if scand(r) else ''
    table('Άνθρωποι και εμπόδια',
          ['Run', 'Χρόνος σε προσωπική ζώνη', 'Path σε προσωπική ζώνη', 'Συνολικό path',
           'Απόσταση από άνθρωπο min / mean [m]', 'Απόσταση από εμπόδιο min / mean [m]'],
          [[f'R{i}', na(r, with_share(r['zone_time_s'], r['zone_time_fraction'], 's')),
            na(r, with_share(r['zone_path_m'], r['zone_path_fraction'], 'm', 2)), num(r['path_length_m'])+' m'+mark(r),
            na(r, pair(r['human_min_m'], r['human_mean_m'])), na(r, pair(r['obstacle_min_m'], r['obstacle_mean_m']))]
           for i, r in enumerate(rows, 1)])
    if any(scand(r) for r in rows):
        out.append('\n‡ SCAND: οι μετρικές ασφάλειας δεν υπολογίζονται. Η θέση του ρομπότ είναι η ηχογράφηση του '
                   'Jackal (έως ~2 m/s), ενώ η κατάσταση που βλέπει ο controller είναι σε κλίμακα iWalk (0.3 m/s): '
                   'φρενάρισμα και αποστάσεις δεν περιγράφουν εφικτή κίνηση του iWalk, και φυσικές επαφές δεν '
                   'υπάρχουν. Στο SCAND έχουν νόημα μόνο οι μετρικές clustering, αντιστοίχισης και επιλογής.\n'
                   '† Το συνολικό path είναι της ηχογράφησης, ίδιο για κάθε controller.')
    table('Προτεινόμενη διαδρομή και παρέμβαση',
          ['Run', 'Απόσταση πρότασης από εμπόδιο min / mean [m]', 'Απόσταση τόξου εντολής χρήστη min / mean [m]',
           'Μήκος πρότασης mean / min [m]', 'Διαφορά καμπυλότητας mean / max [1/m]', 'Λόγος ταχύτητας (επιλογή/εντολή)'],
          [[f'R{i}', na(r, pair(r['proposed_shared_clearance_min_m'], r['proposed_shared_clearance_mean_m'])),
            na(r, pair(r['proposed_human_clearance_min_m'], r['proposed_human_clearance_mean_m'])),
            pair(r['proposed_shared_length_mean_m'], r['proposed_shared_length_min_m']),
            pair(r['intervention_curvature_difference_mean_1pm'], r['intervention_curvature_difference_max_1pm']),
            num(r['intervention_speed_ratio_mean'])] for i, r in enumerate(rows, 1)])
    stats = lambda r, q, d=3: ' / '.join(num(r[f'{q}_{s}_abs'], d) for s in ('mean', 'max', 'min'))
    table('Ομαλότητα κίνησης (|τιμή|: mean / max / min)',
          ['Run', 'Γραμμική επιτ. [m/s²]', 'Γωνιακή επιτ. [rad/s²]', 'Γραμμικό jerk [m/s³]', 'Γωνιακό jerk [rad/s³]'],
          [[f'R{i}', stats(r, 'linear_acceleration'), stats(r, 'angular_acceleration'),
            stats(r, 'linear_jerk'), stats(r, 'angular_jerk')] for i, r in enumerate(rows, 1)])
    cells = []
    for i, r in enumerate(rows, 1):
        for group in sorted({k.split('_')[1] for k in r if k.startswith('dwal_') and k.endswith('_messages')}):
            cells.append([f'R{i}', group, r[f'dwal_{group}_messages'],
                          *(num(r[f'dwal_{group}_{key}'], 0, 100)+'%' if r[f'dwal_{group}_{key}'] is not None else 'n/a'
                            for key, _ in CODES),
                          num(r[f'dwal_{group}_no_cluster'], 0, 100)+'%' if r[f'dwal_{group}_no_cluster'] is not None else 'n/a'])
    table('Κωδικοί διάγνωσης DWAL (μέσο ποσοστό διαδρομών ανά μήνυμα)',
          ['Run', 'Ομάδα', 'Μηνύματα', 'Σε cluster', '-1 άκυρη/φιλτρ.', '-2 φρενάρισμα', '-3 εκτός slice',
           '-4 min span', 'Μηνύματα χωρίς cluster'], cells)
    out.append('\nΣτον fixed DWAL (upstream) ο κωδικός -4 περιλαμβάνει και τα ελεύθερα paths που πετά το clustering '
               'όταν το κενό δεικτών είναι 3-5 (σφάλμα B1), όχι μόνο όσα απορρίπτει το min span.')
    extended(rows, table, out)
    return '\n'.join(out) + '\n'


def extended(rows, table, out):
    """Tables of the offline metrics (schema 6)."""
    pct = lambda v: 'n/a' if v is None else f'{v*100:.0f}%'
    stages = ('reference', 'selected', 'smoothed', 'safe', 'final')
    table('Αλυσίδα εντολών: μέση ταχύτητα ανά στάδιο [m/s]',
          ['Run', *stages, 'Paths με συντελεστή κόστους στο s_min', 'Καθυστέρηση mean / Σp95 [s]'],
          [[f'R{i}', *(num(r.get(f'chain_{k}_mps'), 3) for k in stages),
            ' / '.join(f'{g} {pct(r.get(f"cost_{g}_share_s_min"))}' for g in ('near', 'far')
                       if r.get(f'cost_{g}_share_s_min') is not None) or ('όχι (dynamic)' if r.get('cost_applied') is False else 'n/a'),
            pair(r.get('latency_mean_s'), r.get('latency_p95_sum_s'))] for i, r in enumerate(rows, 1)])
    out.append('\nΤο «safe» (collision monitor) δεν ξαναδημοσιεύει μηδενικές εντολές, οπότε ο μέσος όρος του '
               'αφορά μόνο τις στιγμές που δημοσιεύει. Στον dynamic πριν από τις 2026-11-04 εφαρμοζόταν ακόμα '
               'ο συντελεστής κόστους.')
    out.append('\nΗ καθυστέρηση είναι η ηλικία της πληροφορίας από scan έως /cmd_vel (χρόνοι λήψης στο bag, '
               'βήμα 0.1 s του /clock): μέτρο για τον χρόνο αντίδρασης T του μοντέλου πέδησης.')
    cells = []
    for i, r in enumerate(rows, 1):
        for g in ('near', 'far'):
            if r.get(f'cl_{g}_mixed_cluster_fraction') is None and r.get(f'cl_{g}_flicker_per_min') is None: continue
            cells.append([f'R{i}', g, pct(r.get(f'cl_{g}_mixed_cluster_fraction')), pct(r.get(f'cl_{g}_oversplit_run_fraction')),
                          pair(r.get(f'cl_{g}_flicker_per_min'), r.get(f'cl_{g}_count_changes_per_min'), 1),
                          num(r.get(f'cl_{g}_stability_ari_mean')), pct(r.get(f'cl_{g}_replica_agreement_fraction'))])
    if cells:
        table('Clustering χωρίς ground truth', ['Run', 'Ομάδα', 'Mixed clusters', 'Over-split', 'Flicker / αλλαγές πλήθους [ανά λεπτό]',
                                                'Σταθερότητα (ARI)', 'Συμφωνία αντιγράφου'], cells)
    cells = []
    for i, r in enumerate(rows, 1):
        for g in ('near', 'far'):
            for src, name in (('recorded', 'όπως δημοσιεύτηκαν'), ('replay_upstream', 'replay upstream'), ('replay_tracking', 'replay tracking')):
                if r.get(f'id_{g}_{src}_id_switches_per_min') is None: continue
                k = lambda key: r.get(f'id_{g}_{src}_{key}')
                cells.append([f'R{i}', g, name + (f' ({r.get("correspondence")})' if src == 'recorded' and r.get('correspondence') else ''),
                              num(k('id_switches_per_min'), 1), num(k('fragmentations_per_min'), 1), num(k('crossings_per_min'), 1),
                              num(k('duplicate_id_frames'), 0), num(k('unsupported_inheritance_per_min'), 1)])
    if cells:
        table('Συνέπεια IDs (ανά λεπτό)', ['Run', 'Ομάδα', 'IDs', 'ID switches', 'Fragmentations', 'Crossings',
                                          'Frames με διπλό ID', 'Αστήρικτη κληρονομιά'], cells)
        out.append('\nΤα «replay» εφαρμόζουν offline και τους δύο τρόπους αντιστοίχισης στα ίδια clusters του bag, '
                   'οπότε συγκρίνονται με την ίδια είσοδο.')
    cells = []
    for i, r in enumerate(rows, 1):
        f = lambda key: r.get(f'intent_iwalk_feasible_frames_{key}')
        if f('frames') is None: continue
        cells.append([f'R{i}', num(f('frames'), 0), pair(f('hit_rate'), f('baseline_user_cluster_hit_rate')),
                      pair(f('brier_mean'), f('baseline_uniform_brier_mean')), num(f('logloss_mean')),
                      ' / '.join(num(f(k), 2) for k in ('phi_error_mean_rad', 'baseline_phi_user_error_mean_rad',
                                                        'baseline_reference_curvature_phi_error_mean_rad')),
                      ' / '.join(num(f(k), 2) for k in ('ade_mean_m', 'baseline_ade_mean_m', 'fde_mean_m', 'baseline_fde_mean_m')),
                      pct(r.get('intent_exception_fraction')), pct(r.get('intent_false_stop'))])
    if cells:
        table('Επιλογή έναντι του οδηγού (SCAND, frames εφικτά για iWalk)',
              ['Run', 'Frames', 'Hit rate / baseline', 'Brier / ομοιόμορφο', 'Log-loss',
               '|Δφ| επιλογή / φ_user / κ_ref [rad]', 'ADE επιλογή / baseline, FDE επιλογή / baseline [m]',
               'Εξαιρέσεις iWalk', 'Ψευδή stops'], cells)
        out.append('\nBaseline «μόνο χρήστης»: τόξο με την τωρινή καμπυλότητα ω/v της εντολής. Εξαίρεση: η πραγματική '
                   'διαδρομή του Jackal δεν χωρά το footprint του iWalk (αναφέρεται χωριστά, δεν μπαίνει στον πίνακα).')
    statistics(rows, table, out)


STAT_METRICS = [('standstill_s', 'Ακινησία [s]'), ('collisions', 'Συγκρούσεις'), ('zone_time_s', 'Προσωπική ζώνη [s]'),
                ('human_min_m', 'Min απόσταση ανθρώπου [m]'), ('obstacle_min_m', 'Min απόσταση εμποδίου [m]'),
                ('linear_jerk_mean_abs', 'Γραμμικό jerk [m/s³]'), ('chain_final_mps', 'Μέση ταχύτητα [m/s]')]
# Scenarios with standing people placed beside the robot path (dense_static: y = ±1.0 m, robot on y = 0).
# Every controller spends ~100% of the run in the personal zone at ~1.0 m, so the human metrics there
# describe the layout, not the controller; they are left out of the paired statistics.
LAYOUT_BOUND_HUMAN_SCENARIOS = {'dense_static'}
LAYOUT_BOUND_HUMAN_METRICS = {'zone_time_s', 'human_min_m'}


def wilcoxon(diffs):
    """Wilcoxon signed-rank (zero differences dropped, average ranks for ties): W+, exact two-sided p,
    matched-pairs rank-biserial correlation."""
    d = [x for x in diffs if x != 0]
    n = len(d)
    if n == 0: return None, None, None
    order = sorted(range(n), key=lambda i: abs(d[i]))
    ranks = [0.0]*n
    i = 0
    while i < n:
        j = i
        while j+1 < n and abs(d[order[j+1]]) == abs(d[order[i]]): j += 1
        for k in range(i, j+1): ranks[order[k]] = (i+j)/2 + 1
        i = j+1
    w_plus = sum(r for r, x in zip(ranks, d) if x > 0)
    total = n*(n+1)/2
    # exact null distribution over 2^n sign assignments (ranks doubled to stay integer)
    dist = {0: 1}
    for r in ranks:
        new = {}
        for s, c in dist.items():
            for v in (s, s + int(round(2*r))):
                new[v] = new.get(v, 0) + c
        dist = new
    w2, count = int(round(2*w_plus)), 2**n
    lower = sum(c for s, c in dist.items() if s <= w2)/count
    upper = sum(c for s, c in dist.items() if s >= w2)/count
    return w_plus, min(1.0, 2*min(lower, upper)), (2*w_plus - total)/total


def bootstrap_median(diffs, rounds=10000, seed=0):
    rng = random.Random(seed)
    med = lambda v: sorted(v)[len(v)//2] if len(v) % 2 else sum(sorted(v)[len(v)//2-1:len(v)//2+1])/2
    samples = sorted(med([rng.choice(diffs) for _ in diffs]) for _ in range(rounds))
    return med(diffs), samples[int(0.025*rounds)], samples[int(0.975*rounds)-1]


def statistics(rows, table, out):
    groups = {}
    for r in rows:
        if str(r['scenario']).startswith('scand:'): continue
        key = (r['scenario'], r['seed'], r['semantics'])
        variant = r['variant'] + ('+tracking' if r.get('correspondence') == 'tracking' else '')
        groups.setdefault(key, {})[variant] = r  # latest run per variant (rows are sorted)
    cells = []
    for a, b in (('fixed', 'dynamic_split'), ('dynamic_no_split', 'dynamic_split'),
                 ('dynamic_split', 'dynamic_split+tracking'), ('fixed', 'dynamic_split+tracking')):
        for metric, title in STAT_METRICS:
            diffs = [g[b][metric] - g[a][metric] for key, g in groups.items()
                     if a in g and b in g and g[a].get(metric) is not None and g[b].get(metric) is not None
                     and not (metric in LAYOUT_BOUND_HUMAN_METRICS and key[0] in LAYOUT_BOUND_HUMAN_SCENARIOS)]
            if len(diffs) < 2: continue
            w, p, effect = wilcoxon(diffs)
            med, lo, hi = bootstrap_median(diffs)
            cells.append([f'{b} − {a}', title, len(diffs), f'{med:.3f}', f'[{lo:.3f}, {hi:.3f}]',
                          'n/a' if p is None else f'{p:.3f}', 'n/a' if effect is None else f'{effect:+.2f}'])
    if not cells:
        out.append('\n## Στατιστική\n\nΔεν υπάρχουν ≥ 2 ζεύγη (ίδιο σενάριο, seed, semantics, διαφορετική '
                   'παραλλαγή) για καμία σύγκριση· χρειάζονται runs με περισσότερα seeds ή σενάρια.')
    if cells:
        table('Στατιστική (ζευγαρωτά runs: ίδιο σενάριο, seed, semantics)',
              ['Σύγκριση', 'Μετρική', 'Ζεύγη', 'Διάμεση διαφορά', 'Bootstrap 95% CI', 'Wilcoxon p', 'Rank-biserial r'], cells)
        out.append('\nΔιαφορά = δεύτερο μείον πρώτο. Με λιγότερα από 6 ζεύγη το Wilcoxon δεν μπορεί να δώσει p < 0.05 '
                   '(το μικρότερο δυνατό two-sided p με 5 ζεύγη είναι 0.0625). r = +1: όλα τα ζεύγη προς την ίδια '
                   'θετική κατεύθυνση, 0: καμία τάση. Στις μετρικές «Προσωπική ζώνη» και «Min απόσταση ανθρώπου» '
                   'δεν μετρώνται τα ζεύγη των σεναρίων ' + ', '.join(sorted(LAYOUT_BOUND_HUMAN_SCENARIOS)) +
                   ': οι άνθρωποι στέκονται δίπλα στη διαδρομή (~1.0 m), οπότε κάθε controller είναι ~100% του χρόνου '
                   'στην προσωπική ζώνη και οι τιμές περιγράφουν τη διάταξη, όχι τον controller.')


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('roots', nargs='*', type=Path, default=[REPO/'rosbags'],
                   help='directories searched for runs (default: crowd_aware_ws/rosbags)')
    p.add_argument('--evaluate', action='store_true', help='compute missing or outdated metrics in docker')
    p.add_argument('--force', action='store_true', help='with --evaluate: recompute every run')
    p.add_argument('--image', default='iwalk_dwal:latest')
    p.add_argument('--markdown', type=Path, help='also write the tables to this file')
    p.add_argument('--csv', type=Path, help='also write one row per run with all values')
    a = p.parse_args()
    runs, no_bag = find_runs(a.roots)
    if not runs:
        raise SystemExit(f'no runs (evaluation.json + bag/) under {", ".join(map(str, a.roots))}'
                         + (f'; {len(no_bag)} without bag/' if no_bag else ''))
    if a.evaluate:
        for run in runs:
            data = load(run)
            if a.force or data is None or data.get('schema_version', 0) < SCHEMA: evaluate(run, a.image)
    rows, skipped = [], []
    for run in runs:
        data = load(run)
        if data is None or data.get('schema_version', 0) < SCHEMA: skipped.append(run.name)
        else: rows.append(rows_for(run, data))
    text = markdown(rows) if rows else '# Μετρικές runs\n\nΚανένα run με τρέχουσες μετρικές.\n'
    if skipped:
        text += ('\nΧωρίς τρέχουσες μετρικές (τρέξε με --evaluate): ' + ', '.join(f'`{s}`' for s in skipped) + '\n')
    if no_bag:
        text += (f'\n{len(no_bag)} φάκελοι με evaluation.json αλλά χωρίς bag/ (δεν αξιολογούνται, π.χ. scand_ws '
                 'runs πριν προστεθεί η καταγραφή): ' + ', '.join(f'`{d.name}`' for d in no_bag) + '\n')
    print(text)
    if a.markdown: a.markdown.write_text(text)
    if a.csv and rows:
        fields = list(dict.fromkeys(k for r in rows for k in r))
        with a.csv.open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader(); writer.writerows(rows)


if __name__ == '__main__':
    main()
