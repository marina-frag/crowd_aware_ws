#!/usr/bin/env python3
"""Only collision episodes, requested-motion standstill, and progress to an exit."""
import argparse
import csv
import json
import math
from pathlib import Path
from dwal_bag import events, locate, sample_geometry, twist


class Metrics:
    def __init__(self, cfg, duration):
        self.cfg, self.duration = cfg, duration
        self.start = self.last = self.end = None
        self.ref = self.odom = self.contact = None
        self.initial = self.final = self.final_time = None
        self.requested = self.stopped = self.known_motion = self.contact_coverage = 0.0
        self.pairs = {}; self.episodes = 0
        self.reference_messages = self.odom_messages = self.contact_messages = 0

    def advance(self, now):
        if self.last is not None and now < self.last:
            raise ValueError('Simulation clock moved backwards inside this bag; split the runs')
        if self.start is not None and self.last is not None:
            lo, hi = max(self.last, self.start), min(now, self.start+self.duration)
            if hi > lo:
                if self.ref is not None and self.odom is not None:
                    until = min(hi, self.ref[0]+self.cfg['reference_timeout_s'],
                                self.odom[0]+self.cfg['odom_timeout_s'])
                    dt = max(0.0, until-lo)
                    self.known_motion += dt
                    if abs(self.ref[1]) > self.cfg['reference_motion_threshold_mps']:
                        self.requested += dt
                        if abs(self.odom[1]) < self.cfg['stationary_threshold_mps']:
                            self.stopped += dt
                if self.contact is not None:
                    self.contact_coverage += max(0., min(hi,self.contact+self.cfg['contact_max_gap_s'])-lo)
        self.last = now
        if self.start is not None: self.end = min(now, self.start+self.duration)

    def reference(self, now, speed):
        self.reference_messages += 1
        self.ref = (now, speed)
        if self.start is None and abs(speed) > self.cfg['reference_motion_threshold_mps']:
            if self.odom is not None and now-self.odom[0] <= self.cfg['odom_timeout_s']:
                self.start = self.end = now
                self.initial = self.final = self.odom[2]
                self.final_time = self.odom[0]

    def odometry(self, now, speed, xy):
        self.odom_messages += 1
        if not all(math.isfinite(x) for x in (speed,*xy)): return
        self.odom = (now, speed, xy)
        if self.start is not None and now <= self.start+self.duration:
            self.final = xy; self.final_time = now

    def contacts(self, now, pairs):
        self.contact_messages += 1
        self.contact = now
        if self.start is None or now > self.start+self.duration: return
        for pair in set(tuple(sorted(p)) for p in pairs):
            if any(fragment in name for fragment in self.cfg['ignored_contact_name_fragments'] for name in pair):
                continue
            if pair not in self.pairs or now-self.pairs[pair] > self.cfg['contact_release_s']:
                self.episodes += 1
            self.pairs[pair] = now

    def result(self):
        elapsed = 0. if self.start is None else max(0., self.end-self.start)
        contact_ok = elapsed > 0 and self.contact_coverage/elapsed >= .95
        progress = None
        if (self.initial is not None and self.final is not None and
                self.end-self.final_time <= self.cfg['odom_timeout_s']):
            ex,ey = self.cfg['exit_xy']
            progress = math.hypot(self.initial[0]-ex,self.initial[1]-ey)-math.hypot(self.final[0]-ex,self.final[1]-ey)
        return {
            'condition': {k:self.cfg.get(k) for k in ('controller','variant','semantics','scenario','seed')},
            'duration_requested_s': self.duration, 'duration_observed_s': elapsed,
            'complete_window': elapsed >= self.duration-1e-6,
            'collision_episodes': self.episodes if contact_ok else None,
            'collision_definition': 'physical robot/object contact-pair episodes; ground-name filter; NOT inflated costmap overlap',
            'collision_status': 'contact stream coverage OK; validate human collision shapes in Gazebo before scientific use' if contact_ok else 'unavailable/incomplete contact stream',
            'standstill_while_requested_s': self.stopped if self.known_motion > 0 else None,
            'requested_motion_s': self.requested,
            'progress_to_exit_m': progress,
            'motion_coverage_fraction': self.known_motion/elapsed if elapsed else 0.,
            'contact_coverage_fraction': self.contact_coverage/elapsed if elapsed else 0.,
            'counts': {'reference':self.reference_messages,'odometry':self.odom_messages,'contacts':self.contact_messages},
            'limits': ['Three metrics evaluate navigation outcomes, not clustering ground-truth accuracy.',
                       'Missing/stale intervals are excluded and coverage is reported.',
                       'Compare only equal complete windows and identical reference schedules.']}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('bag');p.add_argument('--duration',type=float)
    p.add_argument('--output');a=p.parse_args();bag,cfg=locate(a.bag)
    duration=a.duration if a.duration is not None else cfg['duration_s']
    if not math.isfinite(duration) or duration<=0:p.error('duration must be positive')
    dest=Path(a.output) if a.output else bag.parent/'analysis';dest.mkdir(parents=True,exist_ok=True)
    t=cfg['topics'];metric=Metrics(cfg,duration)
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
                    raise ValueError('Odometry frame differs from the recorded exit frame')
                metric.odometry(now,msg.twist.twist.linear.x,(msg.pose.pose.position.x,msg.pose.pose.position.y))
            elif topic==t['contacts']:
                metric.contacts(now,[(s.collision1_name,s.collision2_name) for s in msg.states])
            elif topic==t['sampled_paths']:
                for row in sample_geometry(msg,cfg['reference_radius_m']):writer.writerow({'time_s':now,**row})
            else:
                cf.write(json.dumps({'time_s':now,'topic':topic,'message':message_to_ordereddict(msg)})+'\n')
    result=metric.result()
    provenance=bag.parent/'reference_schedule.json'
    result['reference_schedule']=json.loads(provenance.read_text()) if provenance.exists() else None
    (dest/'metrics.json').write_text(json.dumps(result,indent=2,allow_nan=False))
    print(json.dumps(result,indent=2,allow_nan=False))


if __name__=='__main__':main()
