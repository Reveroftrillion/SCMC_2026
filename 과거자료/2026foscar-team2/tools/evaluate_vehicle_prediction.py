#!/usr/bin/env python3
"""Evaluate shadow predictions from synchronized roundabout track CSV.

Usage: python3 tools/evaluate_vehicle_prediction.py TRACK_CSV OUTPUT_DIR
This dataset-specific evaluation defaults to the recorded stop at s879.52
(override with --ego-s) and ego yaw89.4deg. Errors compare later observed
centroids, not ground-truth footprints. lane_smoothed uses a causal primary
candidate; branch_coverage_best is an oracle coverage metric, not accuracy.
Timing includes track updates and occupancy evaluation.
"""
import argparse
import csv
from collections import Counter,defaultdict
import json
import math
from pathlib import Path
import time

import numpy as np
from foscar_control.core import Route
from foscar_planning.state import Context,EventInfo
from foscar_planning.prediction import VehicleTrack,GapPredictor,integrate_turn
from foscar_planning.lane_prediction import LaneMap


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('tracks');parser.add_argument('output')
    parser.add_argument('--lane-map', help='Directed link_set.json in localization map coordinates')
    parser.add_argument('--horizons', type=float, nargs='+', default=[1.,2.,3.])
    parser.add_argument('--ego-s', type=float, default=879.52)
    args=parser.parse_args()
    if any(not math.isfinite(t) or t<=0 or t>8. for t in args.horizons):
        parser.error('horizons must be finite and between 0 and 8 seconds')
    lanes=LaneMap.from_file(args.lane_map) if args.lane_map else None
    out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    rows=list(csv.DictReader(open(args.tracks)))
    events=[];byid=defaultdict(list)
    for r in rows:
        if r['type']!='VEHICLE':continue
        e=EventInfo(type=r['type'],stamp=float(r['t']),x=float(r['mx']),y=float(r['my']),
                    vx=float(r['vx']),vy=float(r['vy']),size_x=float(r['sx']),size_y=float(r['sy']),
                    value='lidar:replay:'+r['id'],box_yaw=math.radians(89.4))
        events.append(e);byid[e.value].append(e)
    if not events:
        parser.error('input contains no VEHICLE observations')
    errors=defaultdict(list); per_track=defaultdict(lambda:defaultdict(list)); matches=Counter()
    for group in byid.values():
        group.sort(key=lambda e:e.stamp);track=VehicleTrack(lane_map=lanes)
        for e in group:
            track.update(e)
            if not track.ready or e.speed<.5:continue
            motion=track.lane_motion
            paths=motion.paths if motion else []
            smooth_speed=motion.speed if motion else e.speed
            if lanes:
                matches['eligible']+=1
                if paths:matches['matched']+=1
            for horizon in args.horizons:
                future=min(group,key=lambda a:abs(a.stamp-e.stamp-horizon))
                if abs(future.stamp-e.stamp-horizon)>.12:continue
                dt=future.stamp-e.stamp
                for name,rate in [('straight',0.),('turn',track.turn_rate)]:
                    x,y,_=integrate_turn(e.x,e.y,track.heading,e.speed,rate,dt)
                    error=math.hypot(x-future.x,y-future.y)
                    errors[f'{name}_{horizon:g}s'].append(error)
                    per_track[e.value][f'{name}_{horizon:g}s'].append(error)
                if paths:
                    for name,speed in [('lane',e.speed),('lane_smoothed',smooth_speed)]:
                        p=paths[0].at(speed*dt)
                        if p is not None:
                            error=math.hypot(p[0]-future.x,p[1]-future.y)
                            errors[f'{name}_{horizon:g}s'].append(error)
                            per_track[e.value][f'{name}_{horizon:g}s'].append(error)
                    candidates=[p.at(smooth_speed*dt) for p in paths]
                    errors_at=[math.hypot(p[0]-future.x,p[1]-future.y) for p in candidates if p is not None]
                    if errors_at:
                        errors[f'branch_coverage_best_{horizon:g}s'].append(min(errors_at))
    route=Route(np.loadtxt(Path(__file__).resolve().parents[1]/'src/foscar_bringup/routes/global_path.txt'))
    predictor=GapPredictor(lane_map=lanes);events.sort(key=lambda e:e.stamp);i=0;decisions=[];cost=[]
    for t in np.arange(min(e.stamp for e in events),max(e.stamp for e in events),.2):
        fresh=[]
        while i<len(events) and events[i].stamp<=t:fresh.append(events[i]);i+=1
        begin=time.perf_counter()
        predictor.observe(fresh,float(t))
        ctx=Context(s=args.ego_s,x=-99.55,y=312.22,yaw=math.radians(89.4),v=0.,now=float(t),route=route)
        result=predictor.evaluate(ctx);cost.append(1000*(time.perf_counter()-begin))
        decisions.append(dict(t=round(float(t),3),**result))
    summary={'source':str(Path(args.tracks).resolve()), 'ego_s':args.ego_s, 'horizons_s':args.horizons, 'lane_matching':dict(matches), 'per_track_errors_m':{key:{k:{'n':len(v),'median':float(np.median(v)), 'p95':float(np.percentile(v,95))} for k,v in values.items()} for key,values in per_track.items()}, 'errors_m':{k:{'n':len(v),'median':float(np.median(v)),'p95':float(np.percentile(v,95))} for k,v in errors.items()},
             'decisions':dict(Counter(r['decision'] for r in decisions)),
             'evaluation_ms':{'median':float(np.median(cost)),'p95':float(np.percentile(cost,95))}}
    (out/'summary.json').write_text(json.dumps(summary,indent=2));(out/'decisions.json').write_text(json.dumps(decisions,indent=2))
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
