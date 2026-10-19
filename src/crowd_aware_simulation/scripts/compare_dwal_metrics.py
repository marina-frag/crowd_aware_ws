#!/usr/bin/env python3
"""Collect the two evaluation metrics, preserving missing data and coverage flags."""
import argparse,csv,json
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('results',nargs='+',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    rows=[]
    for f in a.results:
        d=json.loads(f.read_text());r={**d['condition'],'source':str(f)}
        for k in ['collision_episodes','standstill_while_requested_s','duration_observed_s','complete_window','motion_coverage_fraction','contact_coverage_fraction']:r[k]=d[k]
        r['reference_sha256']=(d.get('reference_schedule') or {}).get('sha256')
        r['reference_completed']=(d.get('reference_schedule') or {}).get('completed')
        r['comparison_ready']=d.get('comparison_ready', False)
        r['window_start_sim_s']=d.get('window_start_sim_s')
        r['duration_requested_s']=d['duration_requested_s']
        rows.append(r)
    fields=list(dict.fromkeys(k for r in rows for k in r))
    with a.output.open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(rows)
    print(f'{len(rows)} runs -> {a.output}. Pair identical seeds AND reference_sha256; window_start_sim_s AND duration_requested_s. comparison_ready includes both coverage checks and replay timing.')

if __name__=='__main__':main()
