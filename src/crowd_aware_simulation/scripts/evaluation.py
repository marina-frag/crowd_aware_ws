#!/usr/bin/env python3
"""Seven evaluation categories, with explicit definitions and per-metric validity."""
import argparse
import csv
import json
import math
from pathlib import Path
from dwal_bag import events, locate, sample_geometry, twist


class Metrics:
    def __init__(self, cfg, duration, start=None):
        self.cfg, self.duration = cfg, duration
        self.start = self.end = start
        self.last = None
        self.ref = self.odom = self.contact = None
        self.requested = self.stopped = self.known_motion = self.contact_coverage = 0.0
        self.reference_active = self.requested_observed = 0.0
        self.pairs = {}; self.episodes = 0
        self.reference_messages = self.odom_messages = self.contact_messages = 0

    def advance(self, now):
        if self.last is not None and now < self.last:
            raise ValueError('Simulation clock moved backwards inside this bag; split the runs')
        if self.start is not None and self.last is not None:
            lo, hi = max(self.last, self.start), min(now, self.start+self.duration)
            if hi > lo:
                # Odometry coverage is independent of whether a command is active.
                odom_until = (min(hi, self.odom[0]+self.cfg['odom_timeout_s'])
                              if self.odom is not None else lo)
                self.known_motion += max(0.0, odom_until-lo)
                if self.ref is not None:
                    ref_until = min(hi, self.ref[0]+self.cfg['reference_timeout_s'])
                    active_dt = max(0.0, ref_until-lo)
                    self.reference_active += active_dt
                    if abs(self.ref[1]) > self.cfg['reference_motion_threshold_mps']:
                        self.requested += active_dt
                        observed_dt = max(0.0, min(ref_until, odom_until)-lo)
                        self.requested_observed += observed_dt
                        if self.odom is not None and abs(self.odom[1]) < self.cfg['stationary_threshold_mps']:
                            self.stopped += observed_dt
                if self.contact is not None:
                    self.contact_coverage += max(0., min(hi,self.contact+self.cfg['contact_max_gap_s'])-lo)
        self.last = now
        if self.start is not None: self.end = max(self.start, min(now, self.start+self.duration))

    def reference(self, now, speed):
        self.reference_messages += 1
        self.ref = (now, speed)
        if self.start is None and abs(speed) > self.cfg['reference_motion_threshold_mps']:
            if self.odom is not None and now-self.odom[0] <= self.cfg['odom_timeout_s']:
                self.start = self.end = now

    def odometry(self, now, speed, xy):
        self.odom_messages += 1
        if not all(math.isfinite(x) for x in (speed,*xy)): return
        self.odom = (now, speed, xy)

    def contacts(self, now, pairs):
        self.contact_messages += 1
        self.contact = now
        if self.start is None or now < self.start or now > self.start+self.duration: return
        robot = self.cfg.get('robot_model_name', 'iwalk')
        # Merge body/wheel/manifold contacts into one robot-object encounter.
        model_pairs = {tuple(sorted(name.split('::')[0] for name in pair)) for pair in pairs}
        for pair in model_pairs:
            if robot not in pair or pair[0] == pair[1]:
                continue
            if any(fragment in name for fragment in self.cfg['ignored_contact_name_fragments'] for name in pair):
                continue
            if pair not in self.pairs or now-self.pairs[pair] > self.cfg['contact_release_s']:
                self.episodes += 1
            self.pairs[pair] = now

    def result(self):
        elapsed = 0. if self.start is None else max(0., self.end-self.start)
        contact_ok = elapsed > 0 and self.contact_coverage/elapsed >= self.cfg.get('minimum_coverage_fraction', .95)
        coverage_min = self.cfg.get('minimum_coverage_fraction', .95)
        request_coverage = self.requested_observed/self.requested if self.requested > 0 else None
        motion_ok = (elapsed > 0 and self.known_motion/elapsed >= coverage_min and
                     (request_coverage is None or request_coverage >= coverage_min))
        return {
            'condition': {k:self.cfg.get(k) for k in ('controller','variant','semantics','scenario','seed')},
            'duration_requested_s': self.duration, 'duration_observed_s': elapsed,
            'complete_window': elapsed >= self.duration-1e-6,
            'collision_episodes': self.episodes if contact_ok else None,
            'collision_definition': 'physical robot/object MODEL-pair episodes; ground-name filter; NOT inflated costmap overlap',
            'collision_status': 'contact stream coverage OK; geometry validation is a separate prerequisite' if contact_ok else 'unavailable/incomplete contact stream',
            'standstill_while_requested_s': self.stopped if motion_ok else None,
            'requested_motion_s': self.requested,
            'requested_motion_coverage_fraction': request_coverage,
            'reference_active_fraction': self.reference_active/elapsed if elapsed else 0.,
            'standstill_definition': 'Time stationary during unexpired nonzero linear reference; expired commands are inactive',
            'motion_coverage_fraction': self.known_motion/elapsed if elapsed else 0.,
            'contact_coverage_fraction': self.contact_coverage/elapsed if elapsed else 0.,
            'counts': {'reference':self.reference_messages,'odometry':self.odom_messages,'contacts':self.contact_messages},
            'limits': ['Two metrics evaluate navigation outcomes, not clustering ground-truth accuracy.',
                       'Missing/stale odometry is excluded and coverage is reported separately.',
                       'Reference expires after reference_timeout_s; expired intervals are not motion requests.',
                       'Replay completion/count/timing checks remain required to validate reference delivery.',
                       'Compare only equal complete windows and identical reference schedules.']}


def evaluate_main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('bag');p.add_argument('--duration',type=float)
    p.add_argument('--output');p.add_argument('--metrics-config');a=p.parse_args();bag,cfg=locate(a.bag)
    duration=a.duration if a.duration is not None else cfg['duration_s']
    if not math.isfinite(duration) or duration<=0:p.error('duration must be positive')
    dest=Path(a.output) if a.output else bag.parent/'analysis_v3';dest.mkdir(parents=True,exist_ok=True)
    provenance=bag.parent/'reference_schedule.json'
    replay=json.loads(provenance.read_text()) if provenance.exists() else None
    start = replay.get('start_sim_s') if replay else None
    t=cfg['topics'];metric=Metrics(cfg,duration,start=start)
    extra=AdditionalMetrics(cfg,load_settings(a.metrics_config))
    selected={t[k] for k in ('reference_cmd','odom','contacts','sampled_paths','clusters_far','clusters_near','far_diagnostics','near_diagnostics')}
    selected.add(t.get('human_states','/human_states'))
    from rosidl_runtime_py.convert import message_to_ordereddict
    with (dest/'samples.csv').open('w',newline='') as sf, (dest/'clusters.jsonl').open('w') as cf:
        fields=['time_s','index','curvature','phi','r','length'];writer=csv.DictWriter(sf,fieldnames=fields);writer.writeheader()
        for now,topic,msg in events(a.bag,selected):
            metric.advance(now)
            if topic==t['reference_cmd']: extra.command(now,msg)
            elif topic==t['odom']: extra.odometry(now,msg)
            elif topic==t.get('human_states','/human_states'): extra.human_states(now,msg)
            elif topic in (t['clusters_far'],t['clusters_near']): extra.group(now,topic,msg)
            if topic==t['reference_cmd']:
                v=twist(msg).linear.x
                if math.isfinite(v):metric.reference(now,v)
            elif topic==t['odom']:
                if msg.header.frame_id != cfg.get('global_frame','odom'):
                    raise ValueError('Odometry frame differs from the configured global frame')
                metric.odometry(now,msg.twist.twist.linear.x,(msg.pose.pose.position.x,msg.pose.pose.position.y))
            elif topic==t['contacts']:
                metric.contacts(now,[(s.collision1_name,s.collision2_name) for s in msg.states])
            elif topic==t['sampled_paths']:
                for row in sample_geometry(msg,cfg['reference_radius_m']):writer.writerow({'time_s':now,**row})
            elif topic != t.get('human_states','/human_states'):
                cf.write(json.dumps({'time_s':now,'topic':topic,'message':message_to_ordereddict(msg)})+'\n')
    result=metric.result()
    result['reference_schedule']=replay
    result['window_start_sim_s']=metric.start
    result['window_source']='replay start' if start is not None else 'first nonzero reference (legacy/pilot)'
    result['reference_count_matches_replay']=bool(replay and metric.reference_messages == replay.get('messages', -2)+1)
    result['comparison_ready']=bool(metric.requested > 0 and result['reference_count_matches_replay'] and replay and replay.get('completed') and replay.get('timing_valid') and
        replay.get('absolute_start') and result['complete_window'] and
        result['collision_episodes'] is not None and result['standstill_while_requested_s'] is not None)
    result['schema_version']=3
    result['additional_metrics'] = (extra.result(metric.start,duration,dest)
        if metric.start is not None else {'all_seven_metrics_available':False,'error':'No evaluation window'})
    result['all_metrics_comparison_ready'] = bool(result['comparison_ready'] and
        result['additional_metrics']['all_seven_metrics_available'])
    result['limits'][0] = 'Navigation outcome metrics do not establish clustering ground-truth accuracy.'
    (dest/'metrics.json').write_text(json.dumps(result,indent=2,allow_nan=False))
    print(json.dumps(result,indent=2,allow_nan=False))


"""Goal-free offline metrics. Uses bag receipt times, not exact sensor synchronization.

ID continuity is a curvature-membership proxy, not identity ground truth.
All thresholds and preprocessing settings are exported with every result.
"""
import bisect
import csv
import hashlib
import json
import math
from pathlib import Path


def load_settings(path=None):
    if path:
        path = Path(path)
    else:
        here = Path(__file__).resolve()
        candidates = (
            here.parents[1] / 'config' / 'additional_metrics.json',
            here.parents[2] / 'share' / 'crowd_aware_simulation' / 'config' / 'additional_metrics.json',
        )
        path = next((candidate for candidate in candidates if candidate.exists()), candidates[0])
    cfg = json.loads(path.read_text())
    for key, value in cfg.items():
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'Invalid additional metric setting: {key}')
    if int(cfg['smoothing_samples']) != cfg['smoothing_samples'] or cfg['smoothing_samples'] % 2 != 1:
        raise ValueError('smoothing_samples must be a positive odd integer')
    for key in ('cluster_jaccard_threshold', 'cluster_overlap_ambiguity_threshold'):
        if cfg[key] > 1: raise ValueError(f'{key} must be <= 1')
    return cfg


def finite(*values):
    return all(math.isfinite(v) for v in values)


def frame(name):
    return name.lstrip('/')


class Series:
    def __init__(self, rows):
        self.rows = rows
        self.times = [r[0] for r in rows]

    def at(self, t, timeout):
        i = bisect.bisect_right(self.times, t+1e-9)-1
        return self.rows[i] if i >= 0 and t-self.rows[i][0] <= timeout+1e-9 else None


def smooth(values, width):
    half = width//2
    out = [None]*len(values)
    for i in range(half, len(values)-half):
        window = values[i-half:i+half+1]
        if all(x is not None for x in window): out[i] = sum(window)/width
    return out


def derivative(values, step):
    return [None if a is None or b is None else (b-a)/step
            for a,b in zip(values, values[1:])]


def absolute_mean(values, expected, minimum):
    valid = [abs(x) for x in values if x is not None and math.isfinite(x)]
    coverage = len(valid)/expected if expected > 0 else 0.
    return (sum(valid)/len(valid) if valid and coverage >= minimum else None), coverage


def oscillations(times, omega, refs, settings):
    """Count significant opposite lobes during continuously active, stable references.

    This intentionally conservative operational definition cannot prove intent.
    Neutral gaps <= max_reversal are allowed; stale odom/reference breaks continuity.
    """
    threshold = settings['oscillation_omega_threshold_radps']
    step = settings['grid_step_s']
    lobes = []
    current = None
    for t,w in zip(times, omega):
        sign = 0 if w is None or abs(w) < threshold else (1 if w > 0 else -1)
        if current is not None and (not sign or sign != current[2]):
            if current[1]-current[0]+step >= settings['oscillation_min_lobe_s']-1e-9:
                lobes.append(tuple(current))
            current = None
        if sign:
            if current is None: current = [t,t,sign]
            else: current[1] = t
    if current and current[1]-current[0]+step >= settings['oscillation_min_lobe_s']-1e-9:
        lobes.append(tuple(current))
    count = 0
    for left,right in zip(lobes,lobes[1:]):
        if left[2] == right[2] or right[0]-left[1] > settings['oscillation_max_reversal_s']+1e-9:
            continue
        pad = settings['oscillation_reference_padding_s']
        lo = bisect.bisect_left(times,left[0]-pad)
        hi = bisect.bisect_right(times,right[1]+pad)
        # Do not claim stable intent at truncated window boundaries.
        if left[0]-pad < times[0] or right[1]+pad > times[-1]+step: continue
        window = refs[lo:hi]
        if not window or any(r is None for r in window): continue
        if any(w is None for w in omega[lo:hi]): continue
        angular = [r[2] for r in window]
        linear = [r[1] for r in window]
        if (max(angular)-min(angular) <= settings['oscillation_reference_variation_radps'] and
                min(linear)*max(linear) >= 0):
            count += 1
    return count


def cluster_continuity(rows, start, duration, settings):
    changes = eligible = ambiguous = births = deaths = pairs = 0
    covered = 0.
    rows = [(t,g) for t,g in rows if start <= t < start+duration]
    for (ta,a),(tb,b) in zip(rows,rows[1:]):
        dt = tb-ta
        if dt <= 0 or dt > settings['cluster_max_gap_s']: continue
        covered += dt
        pairs += 1
        edges = []
        for i,(_,sa) in enumerate(a):
            for j,(_,sb) in enumerate(b):
                score = len(sa & sb)/len(sa | sb) if sa | sb else 0.
                if score >= settings['cluster_overlap_ambiguity_threshold']:
                    edges.append((i,j,score))
        adegree = [sum(i==x for x,_,_ in edges) for i in range(len(a))]
        bdegree = [sum(j==y for _,y,_ in edges) for j in range(len(b))]
        births += sum(d==0 for d in bdegree)
        deaths += sum(d==0 for d in adegree)
        for i,j,score in edges:
            if adegree[i]==1 and bdegree[j]==1 and score >= settings['cluster_jaccard_threshold']:
                eligible += 1
                changes += a[i][0] != b[j][0]
            else: ambiguous += 1
    return {
        'id_changes': changes if eligible else None,
        'eligible_associations': eligible,
        'id_change_fraction': changes/eligible if eligible else None,
        'ambiguous_overlap_edges_excluded': ambiguous,
        'unmatched_births': births, 'unmatched_deaths': deaths,
        'valid_frame_pairs': pairs,
        'frame_pair_coverage_fraction': min(1.,covered/duration),
        'definition': 'Unique mutual curvature-set overlap (Jaccard); split/merge-like overlaps excluded. Continuity proxy, not tracking ground truth.'}


class AdditionalMetrics:
    def __init__(self, cfg, settings):
        self.cfg, self.settings = cfg, settings
        self.odom = []; self.reference = []; self.humans = []
        self.clusters = {}; self.rejected_frames = {}

    def odometry(self, now, msg):
        p = msg.pose.pose.position
        v,w = msg.twist.twist.linear.x,msg.twist.twist.angular.z
        if frame(msg.header.frame_id) != frame(self.cfg.get('global_frame','odom')):
            self.rejected_frames['odom'] = self.rejected_frames.get('odom',0)+1
            return
        if finite(v,w,p.x,p.y): self.odom.append((now,v,w,p.x,p.y))

    def command(self, now, msg):
        from dwal_bag import twist
        msg = twist(msg)
        if finite(msg.linear.x,msg.angular.z):
            self.reference.append((now,msg.linear.x,msg.angular.z))

    def human_states(self, now, msg):
        if frame(msg.header.frame_id) != frame(self.cfg.get('global_frame','odom')):
            self.rejected_frames['human_states'] = self.rejected_frames.get('human_states',0)+1
            return
        centers = []
        for agent in msg.agents:
            p = agent.position.position
            if not finite(p.x,p.y): return  # invalid human is not an empty crowd
            centers.append((p.x,p.y))
        self.humans.append((now,centers))

    def group(self, now, topic, msg):
        groups = []
        for cluster in msg.clusters:
            members = {round(float(path.curvature),8) for path in cluster.paths
                       if math.isfinite(path.curvature)}
            if members: groups.append((int(cluster.id),members))
        self.clusters.setdefault(topic,[]).append((now,groups))

    def result(self, start, duration, dest):
        cfg,s = self.cfg,self.settings
        minimum = cfg.get('minimum_coverage_fraction',.95)
        dt = s['grid_step_s']
        times = []; weights = []
        n = int(math.ceil(duration/dt))
        for i in range(n):
            times.append(start+i*dt)
            weights.append(min(dt,start+duration-times[-1]))
        odom,human,ref = Series(self.odom),Series(self.humans),Series(self.reference)
        velocities = []; angular = []; references = []; distances = []
        human_coverage = active_human = zone = 0.
        for t,weight in zip(times,weights):
            o = odom.at(t,cfg['odom_timeout_s'])
            h = human.at(t,s['human_timeout_s'])
            r = ref.at(t,cfg['reference_timeout_s'])
            velocities.append(o[1] if o else None)
            angular.append(o[2] if o else None)
            references.append(r)
            d = None
            if o is not None and h is not None:
                human_coverage += weight
                if h[1]:
                    active_human += weight
                    d = min(math.hypot(o[3]-x,o[4]-y) for x,y in h[1])
                    if d < s['personal_zone_center_distance_m']: zone += weight
            distances.append(d)
        width = s['smoothing_samples']
        sv,sw = smooth(velocities,width),smooth(angular,width)
        av,aw = derivative(sv,dt),derivative(sw,dt)
        jv,jw = derivative(av,dt),derivative(aw,dt)
        smooth_results = {}
        for name,values,expected in (
            ('linear_acceleration_mean_abs_mps2',av,n-width),
            ('angular_acceleration_mean_abs_radps2',aw,n-width),
            ('linear_jerk_mean_abs_mps3',jv,n-width-1),
            ('angular_jerk_mean_abs_radps3',jw,n-width-1)):
            value,coverage = absolute_mean(values,expected,minimum)
            smooth_results[name] = value
            smooth_results[name+'_coverage_fraction'] = coverage
        motion_coverage = sum(w for w,v in zip(weights,velocities) if v is not None)/duration
        ref_coverage = sum(w for w,r in zip(weights,references) if r is not None)/duration
        exposure_coverage = human_coverage/duration
        human_ok = exposure_coverage >= minimum
        # Minimum over all observed human/odom update times, not only the 10 Hz grid.
        known_distances = []
        observation_times = sorted(set(odom.times+human.times))
        for t in observation_times:
            if not start <= t < start+duration: continue
            o,h = odom.at(t,cfg['odom_timeout_s']),human.at(t,s['human_timeout_s'])
            if o is not None and h is not None and h[1]:
                known_distances.append(min(math.hypot(o[3]-x,o[4]-y) for x,y in h[1]))
        # Absent references mean no active intent; delivery validation remains in base evaluator.
        osc = oscillations(times,sw,references,s) if motion_coverage >= minimum else None
        temporal = {topic:cluster_continuity(rows,start,duration,s)
                    for topic,rows in self.clusters.items()}
        topics = cfg.get('topics',{})
        expected = [topics.get('clusters_far','/dwal_planner/clusters_far')]
        if cfg.get('controller') == 'fixed_dwal':
            expected.append(topics.get('clusters_near','/dwal_planner/clusters_near'))
        for topic in expected:
            if topic not in temporal:
                temporal[topic] = cluster_continuity([],start,duration,s)
        for value in temporal.values():
            if value['frame_pair_coverage_fraction'] < minimum:
                value['id_changes'] = value['id_change_fraction'] = None
        result = {
            'settings': s,
            'settings_sha256': hashlib.sha256(json.dumps(s,sort_keys=True).encode()).hexdigest(),
            'time_alignment': 'Uniform simulation-time grid; latest bag-received sample held only until timeout. No TF inference.',
            'personal_zone_time_s': zone if human_ok else None,
            'personal_zone_fraction': zone/human_coverage if human_ok and human_coverage else None,
            'minimum_human_center_distance_m': min(known_distances) if human_ok and known_distances else None,
            'human_geometry_definition': 'Planar odometry-origin to HuNav-agent-center distance. Minimum at all observed odom/human update times with bounded hold; NOT footprint clearance or continuous-time minimum.',
            'human_joint_coverage_fraction': exposure_coverage,
            'observed_time_with_humans_s': active_human,
            'rejected_frame_messages': self.rejected_frames,
            'steering_oscillations': osc,
            'oscillation_definition': 'Opposite significant omega lobes during fresh, angularly stable reference; conservative proxy, not proven unrequested motion.',
            'reference_active_grid_fraction': ref_coverage,
            'smoothness': smooth_results,
            'cluster_id_stability': temporal,
            'limits': [
                'Higher clearance/lower proximity exposure is not automatically better in dense crowds.',
                'Lower acceleration/jerk alone can reward standing still; interpret with standstill and safety.',
                'Common downstream smoothing and collision monitoring can mask controller differences.',
                'No goal-based path irregularity is computed.',
                'Curvature-overlap associations are not physical passage identity ground truth; no cross-variant ID comparison.',
                'No human samples means unavailable; valid empty crowd means zero exposure and undefined minimum distance.'
            ]}
        with (Path(dest)/'additional_timeline.csv').open('w',newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['time_s','dt_s','linear_velocity_mps','angular_velocity_radps','human_center_distance_m','reference_linear_mps','reference_angular_radps'])
            for t,w,v,a,d,r in zip(times,weights,velocities,angular,distances,references):
                writer.writerow([t,w,v,a,d,r[1] if r else None,r[2] if r else None])
        result['all_seven_metrics_available'] = bool(human_ok and known_distances and osc is not None and
            all(smooth_results[k] is not None for k in smooth_results if not k.endswith('coverage_fraction')) and
            temporal and all(v['id_changes'] is not None for v in temporal.values()))
        return result
#!/usr/bin/env python3
"""Export all metrics and validity flags without treating unavailable values as zero."""
import argparse
import csv
import json
from pathlib import Path


def flatten(obj, prefix=''):
    result = {}
    for key,value in obj.items():
        name = prefix+key
        if isinstance(value,dict): result.update(flatten(value,name+'.'))
        elif not isinstance(value,list): result[name]=value
    return result


def compare_main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('results',nargs='+',type=Path)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    rows=[]
    for path in a.results:
        if path.is_dir():
            path = path/'analysis_v3'/'metrics.json'
        if path.name != 'metrics.json':
            raise SystemExit(f'comparison input must be metrics.json or a run directory: {path}')
        d=json.loads(path.read_text())
        row=flatten(d)
        row['source']=str(path)
        rows.append(row)
    fields=list(dict.fromkeys(k for row in rows for k in row))
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields)
        writer.writeheader();writer.writerows(rows)
    print(f'{len(rows)} runs -> {a.output}. Pair equal seed, reference SHA, window and additional settings SHA. Check per-metric coverage; ID continuity is a proxy.')

def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest='command', required=True)
    evaluate = subparsers.add_parser('evaluate')
    evaluate.add_argument('bag')
    evaluate.add_argument('--duration', type=float)
    evaluate.add_argument('--output')
    evaluate.add_argument('--metrics-config')
    compare = subparsers.add_parser('compare')
    compare.add_argument('results', nargs='+', type=Path)
    compare.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.command == 'evaluate':
        import sys
        sys.argv = [sys.argv[0], args.bag]
        if args.duration is not None: sys.argv += ['--duration', str(args.duration)]
        if args.output is not None: sys.argv += ['--output', args.output]
        if args.metrics_config is not None: sys.argv += ['--metrics-config', args.metrics_config]
        evaluate_main()
    else:
        import sys
        sys.argv = [sys.argv[0], *[str(path) for path in args.results], '--output', str(args.output)]
        compare_main()

if __name__ == '__main__':
    main()
