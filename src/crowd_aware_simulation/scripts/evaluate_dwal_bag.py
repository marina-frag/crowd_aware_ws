#!/usr/bin/env python3
"""Only collision episodes and standstill while linear motion is requested."""
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


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('bag');p.add_argument('--duration',type=float)
    p.add_argument('--output');a=p.parse_args();bag,cfg=locate(a.bag)
    duration=a.duration if a.duration is not None else cfg['duration_s']
    if not math.isfinite(duration) or duration<=0:p.error('duration must be positive')
    dest=Path(a.output) if a.output else bag.parent/'analysis';dest.mkdir(parents=True,exist_ok=True)
    provenance=bag.parent/'reference_schedule.json'
    replay=json.loads(provenance.read_text()) if provenance.exists() else None
    start = replay.get('start_sim_s') if replay else None
    t=cfg['topics'];metric=Metrics(cfg,duration,start=start)
    selected={t[k] for k in ('reference_cmd','odom','contacts','sampled_paths','clusters_far','clusters_near','far_diagnostics','near_diagnostics')}
    from rosidl_runtime_py.convert import message_to_ordereddict
    with (dest/'samples.csv').open('w',newline='') as sf, (dest/'clusters.jsonl').open('w') as cf:
        fields=['time_s','index','curvature','phi','r','length'];writer=csv.DictWriter(sf,fieldnames=fields);writer.writeheader()
        for now,topic,msg in events(a.bag,selected):
            metric.advance(now)
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
            else:
                cf.write(json.dumps({'time_s':now,'topic':topic,'message':message_to_ordereddict(msg)})+'\n')
    result=metric.result()
    result['reference_schedule']=replay
    result['window_start_sim_s']=metric.start
    result['window_source']='replay start' if start is not None else 'first nonzero reference (legacy/pilot)'
    result['reference_count_matches_replay']=bool(replay and metric.reference_messages == replay.get('messages', -2)+1)
    result['comparison_ready']=bool(metric.requested > 0 and result['reference_count_matches_replay'] and replay and replay.get('completed') and replay.get('timing_valid') and
        replay.get('absolute_start') and result['complete_window'] and
        result['collision_episodes'] is not None and result['standstill_while_requested_s'] is not None)
    (dest/'metrics.json').write_text(json.dumps(result,indent=2,allow_nan=False))
    print(json.dumps(result,indent=2,allow_nan=False))


if __name__=='__main__':main()
