#!/usr/bin/env python3
"""Read-only CSV evidence summary. It NEVER enables control or certifies bounds."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from foscar_control.mission_config import load_mission_config


def stats(values):
    a=np.asarray(values,dtype=float)
    return dict(count=len(a),min=float(a.min()),median=float(np.median(a)),max=float(a.max())) if len(a) else dict(count=0,min=None,median=None,max=None)


def summarize_csv(filename,*,start_s,end_s,brake_min):
    path=Path(filename);rows=[]
    with path.open() as f:
        for row in csv.DictReader(f):
            try:
                item={k:float(row[k]) for k in ('elapsed_s','s_m','speed_mps','brake')}
                if all(math.isfinite(v) for v in item.values()):rows.append(item)
            except (KeyError,ValueError):continue
    selected=[r for r in rows if start_s<=r['s_m']<end_s]
    decels=[];intervals=[]
    for a,b in zip(selected,selected[1:]):
        dt=b['elapsed_s']-a['elapsed_s']
        if dt<=0:continue
        intervals.append(dt)
        if a['brake']>=brake_min and b['brake']>=brake_min:
            decel=(a['speed_mps']-b['speed_mps'])/dt
            if decel>0:decels.append(decel)
    return dict(verified=False,source=str(path.resolve()),source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        highway_interval_m=[start_s,end_s],highway_speed_mps=stats([r['speed_mps'] for r in selected]),
        sample_interval_s=stats(intervals),braking_decel_mps2=stats(decels),brake_selection_min=brake_min,
        requires_measurement=['rear_range_m','front_range_m','latency_s','position_error_m','velocity_error_mps',
                              'min_brake_mps2','max_accel_mps2','lat_accel_mps2','npc_speed_bound_mps'],
        limitations=['Observed extrema are not guaranteed bounds.',
                     'CSV has no side/rear lidar coverage or collision ground truth.',
                     'Sampling gaps and braking transients affect finite differences.'])


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('csv');ap.add_argument('--mission-config',required=True);ap.add_argument('--mission',default='HIGHWAY')
    ap.add_argument('--brake-min',type=float,default=.1);ap.add_argument('--output',required=True)
    args=ap.parse_args();config=load_mission_config(args.mission_config)
    zone=next(z for z in config['zones'] if z['name']==args.mission)
    report=summarize_csv(args.csv,start_s=zone['start_s'],end_s=zone['end_s'],brake_min=args.brake_min)
    Path(args.output).write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(report,indent=2,ensure_ascii=False))

if __name__=='__main__':main()
