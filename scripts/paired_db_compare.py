#!/usr/bin/env python3
"""Exploratory paired comparison: DBSCAN eps = 0.15 (step compare_db) vs the locked jump tau = 0.3
(dynamic_split + tracking runs of step compare). Pairs share scenario, density, path, seed and
semantics. Reads the two summary CSVs; does not change the locked choice of tau = 0.3.

The two batches ran on different Docker images (compare: 2026-11-19/20, compare_db: 2026-11-24),
so this is not a controlled comparison of the clustering alone.
Goals are compared with an exact McNemar test on the discordant pairs.
"""
import argparse
import csv
import sys
from math import comb
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from metrics_summary import LOSS_PEAKS, bootstrap_mean, bootstrap_median, holm, num, wilcoxon  # noqa: E402

KEY = ('scenario', 'density', 'path_shape', 'seed', 'semantics')
LOSS_KEYS = ('cl_far_choice_violation_fraction', 'cl_far_completeness_violation_fraction',
             'id_far_recorded_id_switches_per_min')
METRICS = [('standstill_s', 'Ακινησία [s]'), ('zone_time_s', 'Προσωπική ζώνη [s]'),
           ('human_min_m', 'Min απόσταση ανθρώπου [m]'), ('obstacle_min_m', 'Min απόσταση εμποδίου [m]'),
           ('pd_time_to_goal_s', 'Χρόνος ως τον στόχο [s]'), ('chain_final_mps', 'Μέση ταχύτητα [m/s]'),
           ('linear_jerk_mean_abs', 'Γραμμικό jerk [m/s³]'),
           ('cl_far_choice_violation_fraction', 'U: underclustering'),
           ('cl_far_completeness_violation_fraction', 'O: overclustering'),
           ('id_far_recorded_id_switches_per_min', 'S: ID switches / min'), ('loss', 'Loss (ανά run)')]
LAYOUT_BOUND = {'dense_static': {'zone_time_s', 'human_min_m'}}  # same exclusion as metrics_summary


def value(text):
    if text in ('', 'None', 'nan'): return None
    if text in ('True', 'False'): return text == 'True'
    try: return float(text)
    except ValueError: return text


def load(path, keep):
    rows = {}
    for r in csv.DictReader(open(path, encoding='utf-8')):
        r = {k: value(v) for k, v in r.items()}
        if not keep(r): continue
        parts = [r.get(k) for k in LOSS_KEYS]
        r['loss'] = None if None in parts else sum(v / p for v, p in zip(parts, LOSS_PEAKS))
        key = tuple(r.get(k) for k in KEY)
        if key in rows: raise SystemExit(f'duplicate run for {key} in {path}')
        rows[key] = r
    return rows


def mean(values):
    values = [v for v in values if v is not None]
    return sum(values) / len(values) if values else None


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--tau', default='rosbags/results/summary_compare.csv')
    p.add_argument('--db', default='rosbags/results/summary_compare_db.csv')
    p.add_argument('--markdown', default='rosbags/results/paired_db_vs_tau.md')
    p.add_argument('--same-image', action='store_true',
                   help='both batches ran on the same iwalk_dwal image (e.g. compare_tau_doorway vs compare_db)')
    a = p.parse_args()
    tau = load(a.tau, lambda r: r.get('variant') == 'dynamic_split' and r.get('correspondence') == 'tracking'
               and r.get('jump') == 0.3 and r.get('split_method') in (None, 'jump'))
    db = load(a.db, lambda r: r.get('variant') == 'dynamic_split' and r.get('correspondence') == 'tracking'
              and r.get('split_method') == 'dbscan' and r.get('eps') == 0.15)
    keys = sorted(set(tau) & set(db), key=str)
    out = ['# Ζευγαρωτή σύγκριση: DBSCAN ε = 0.15 − jump τ = 0.3 (διερευνητική)', '',
           f'Runs: τ = 0.3 {len(tau)}, ε = 0.15 {len(db)}, ζεύγη {len(keys)} '
           f'(χωρίς ζεύγος: τ {len(set(tau) - set(db))}, ε {len(set(db) - set(tau))}). '
           'Ζεύγος = ίδιο σενάριο, πυκνότητα, διαδρομή, seed, semantics.', '',
           ('Τα δύο batches έτρεξαν στο ίδιο Docker image, άρα η διαφορά οφείλεται στο clustering. '
            if a.same_image else
            '**Όριο:** τα δύο batches έτρεξαν σε διαφορετικό Docker image, άρα η διαφορά δεν οφείλεται '
            'αποδεδειγμένα μόνο στο clustering. ') + 'Το τ = 0.3 μένει κλειδωμένο.', '']
    same_agents = sum(tau[k].get('agents_sha256') == db[k].get('agents_sha256') for k in keys)
    out.append(f'Ίδιοι agents στο ζεύγος: {same_agents}/{len(keys)}.')
    out.append('')
    out += ['| | τ = 0.3 | ε = 0.15 |', '|---|---|---|']
    for label, f in (('Runs με σύγκρουση', lambda r: r.get('collisions') not in (0, 0.0)),
                     ('Στόχος', lambda r: r.get('pd_goal_reached') is True)):
        out.append(f'| {label} | {sum(f(tau[k]) for k in keys)} | {sum(f(db[k]) for k in keys)} |')
    goal = lambda r: r.get('pd_goal_reached') is True
    only_db = [k for k in keys if goal(db[k]) and not goal(tau[k])]
    only_tau = [k for k in keys if goal(tau[k]) and not goal(db[k])]
    n, low = len(only_db) + len(only_tau), min(len(only_db), len(only_tau))
    mcnemar = min(1.0, 2 * sum(comb(n, i) for i in range(low + 1)) / 2 ** n) if n else None
    out.append(f'| Στόχος μόνο με αυτό (McNemar exact p = {num(mcnemar, 4)}) | {len(only_tau)} | {len(only_db)} |')
    for name in ('loss',) + LOSS_KEYS:
        out.append(f'| Μέσο {name} | {num(mean(tau[k][name] for k in keys), 3)} | {num(mean(db[k][name] for k in keys), 3)} |')
    out += ['', '## Ζευγαρωτά τεστ (ε − τ, Wilcoxon, Holm σε όλες τις γραμμές)', '',
            '| Μετρική | Ζεύγη | Μέσο τ | Μέσο ε | Διάμεση διαφορά [95% CI] | Μέση διαφορά [95% CI] | p | r | Holm p |',
            '|---|---|---|---|---|---|---|---|---|']
    rows, pvalues = [], []
    for key, label in METRICS:
        pairs = [(tau[k].get(key), db[k].get(key)) for k in keys
                 if key not in LAYOUT_BOUND.get(k[0], ())]
        pairs = [(x, y) for x, y in pairs if x is not None and y is not None]
        if len(pairs) < 2:
            rows.append([label, len(pairs)] + ['n/a'] * 6); pvalues.append(None); continue
        diffs = [y - x for x, y in pairs]
        md, ml, mh = bootstrap_median(diffs)
        mn, nl, nh = bootstrap_mean(diffs)
        _, pv, r = wilcoxon(diffs)
        rows.append([label, len(pairs), num(mean(x for x, _ in pairs), 3), num(mean(y for _, y in pairs), 3),
                     f'{num(md, 3)} [{num(ml, 3)}, {num(mh, 3)}]', f'{num(mn, 3)} [{num(nl, 3)}, {num(nh, 3)}]',
                     num(pv, 4), 'n/a' if r is None else f'{r:+.2f}'])
        pvalues.append(pv)
    for row, h in zip(rows, holm(pvalues)):
        out.append('| ' + ' | '.join(str(c) for c in row) + f' | {num(h, 4)} |')
    out += ['', 'Αρνητική διαφορά = μικρότερη τιμή για το ε = 0.15. Για αποστάσεις και ταχύτητα το μεγαλύτερο είναι '
            'καλύτερο. Στο dense_static οι μετρικές ανθρώπων εξαιρούνται (όπως στο metrics_summary). '
            'Το Loss ανά run χρησιμοποιεί τους παγωμένους παρονομαστές του βήματος 2.', '',
            '## Ανά σενάριο (μέσοι όροι)', '',
            '| Σενάριο | Ζεύγη | Στόχοι τ / ε | Loss τ / ε | Ακινησία τ / ε [s] | Min εμποδίου τ / ε [m] |',
            '|---|---|---|---|---|---|']
    for scen in sorted({k[0] for k in keys}):
        ks = [k for k in keys if k[0] == scen]
        g = lambda rows_, f: sum(f(rows_[k]) for k in ks)
        m = lambda rows_, name: num(mean(rows_[k].get(name) for k in ks), 3)
        out.append(f'| {scen} | {len(ks)} | {g(tau, lambda r: r.get("pd_goal_reached") is True)} / '
                   f'{g(db, lambda r: r.get("pd_goal_reached") is True)} | {m(tau, "loss")} / {m(db, "loss")} | '
                   f'{m(tau, "standstill_s")} / {m(db, "standstill_s")} | {m(tau, "obstacle_min_m")} / {m(db, "obstacle_min_m")} |')
    Path(a.markdown).write_text('\n'.join(out) + '\n', encoding='utf-8')
    print('\n'.join(out))


if __name__ == '__main__':
    main()
