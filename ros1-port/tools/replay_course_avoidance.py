#!/usr/bin/env python3
"""Offline continuous recorded-footprint replay of STATIC_OBSTACLE avoidance.

No ROS master/launch/control sockets. Evaluate at the recorded target ticks;
positions remain the recorded trajectory, so this cannot certify closed-loop
avoidance. --baseline-head compares the immutable pre-fix Avoid implementation.
"""
import argparse
from collections import Counter
import importlib
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import rosbag
import yaml

PORT=Path(__file__).resolve().parents[1]
sys.dont_write_bytecode=True


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bag',required=True,type=Path)
    ap.add_argument('--output',required=True,type=Path)
    ap.add_argument('--baseline-head',action='store_true')
    ap.add_argument('--stop-at',type=float,default=math.inf)
    args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    from ctrl_control.ctrl_core import Route
    from ctrl_planning.ctrl_state import Context
    from ctrl_planning.ctrl_signal_stop import SignalStopPolicy
    from ctrl_planning.ctrl_lane_prediction import LaneMap
    if args.baseline_head:
        key='ctrl_planning.states.ctrl_avoid'
        source=subprocess.check_output(['git','show','HEAD:ros1-port/src/ctrl_planning/ctrl_planning/states/ctrl_avoid.py'],cwd=PORT.parent)
        module=importlib.util.module_from_spec(importlib.util.spec_from_loader(key,loader=None));sys.modules[key]=module
        exec(compile(source,'<HEAD:ctrl_avoid.py>','exec'),module.__dict__)
    from ctrl_planning.states.ctrl_avoid import Avoid
    route=Route(np.loadtxt(PORT/'src/ctrl_bringup/routes/global_path.txt'))
    road=LaneMap(json.loads((PORT/'src/ctrl_bringup/maps/link_set.json').read_text()))
    config=yaml.safe_load((PORT/'src/ctrl_planning/config/behavior.yaml').read_text())['behavior_node']['ros__parameters']
    avoid=Avoid(**{k[6:]:v for k,v in config.items() if k.startswith('avoid.')})
    policy=SignalStopPolicy(json.loads((PORT/'src/ctrl_camera/config/yolo_stoplines.json').read_text())['lines'])
    pose=None;s=None;fp=None;mission='';rows=[];poses=[];began=time.monotonic()
    selected=['/localization/odometry','/localization/s','/perception/footprints','/perception/traffic_signal','/behavior/target','/behavior/mission']
    with rosbag.Bag(str(args.bag)) as bag:
        for topic,msg,t in bag.read_messages(topics=selected):
            now=t.to_sec()
            if now>args.stop_at:break
            if topic=='/localization/odometry':
                p=msg.pose.pose.position;q=msg.pose.pose.orientation
                pose=(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),msg.twist.twist.linear.x)
                poses.append((msg.header.stamp.to_sec(),pose));poses=poses[-150:]
            elif topic=='/localization/s':s=msg.data
            elif topic=='/perception/footprints':fp=json.loads(msg.data)
            elif topic=='/behavior/mission':mission=json.loads(msg.data)['mission']
            elif topic=='/perception/traffic_signal' and s is not None:
                d=json.loads(msg.data);policy.observe(d['id'],d['color'],s,d['stamp'],now)
            elif topic=='/behavior/target' and mission=='STATIC_OBSTACLE' and pose is not None and s is not None and fp:
                ctx=Context(s=s,x=pose[0],y=pose[1],yaw=pose[2],v=pose[3],now=now,route=route,road_map=road,mission_name=mission)
                policy.current(s);ctx.lateral_hold_s=policy.lateral_hold_s(s)
                ctx.occupancy_stamp=fp['stamp'];ctx.occupancy_hulls={o['id']:o['xy'] for o in fp['objects']}
                ctx.occupancy_heights={o['id']:o['height'] for o in fp['objects']}
                ctx.occupancy_kinds={o['id']:o['kind'] for o in fp['objects']}
                ctx.occupancy_speeds={o['id']:(o['vx'],o['vy']) for o in fp['objects']}
                ctx.occupancy_cleared=fp['cleared']
                scan_pose=min(poses,key=lambda p:abs(p[0]-fp['stamp']))[1]
                ctx.occupancy_sensor=scan_pose[:2]
                before=time.monotonic();entered=avoid.check_entry(ctx)
                target=avoid.compute_target(ctx) if entered else None
                if target is not None:target=policy.apply(target,s,now)
                boxes=avoid.boxes()
                row=dict(t=now,s=s,hold_s=ctx.lateral_hold_s,nearest_box_s=min((b.s_min for b in boxes),default=None),
                    recorded=dict(offset=msg.lat_offset,v_max=msg.v_max,state=msg.state),
                    replay=None if target is None else dict(offset=target.lat_offset,v_max=target.v_max,state=target.state,
                        stop_s=target.stop_s,entry_s=target.entry_s,entry_length_m=target.entry_length_m,chain=target.chain),
                    lateral_started=avoid.lateral_started,reason=avoid.bev.last_reason if avoid.bev else None,
                    calculation_s=time.monotonic()-before)
                rows.append(row)
                if len(rows)%100==0:print('ticks',len(rows),'s',s,'offset',None if target is None else target.lat_offset,flush=True)
    def clean(v):
        if isinstance(v,float) and not math.isfinite(v):return None
        if isinstance(v,dict):return {k:clean(x) for k,x in v.items()}
        if isinstance(v,(tuple,list)):return [clean(x) for x in v]
        return v
    (args.output/'ticks.json').write_text(json.dumps(clean(rows),indent=2)+'\n')
    summary=dict(ticks=len(rows),elapsed_s=time.monotonic()-began,baseline_head=args.baseline_head,
        reasons=dict(Counter(r['reason'] for r in rows)),offset_nonzero=sum(abs(r['replay']['offset'])>1e-6 for r in rows if r['replay']),
        zero_cap_ticks=sum(r['replay']['v_max']==0 for r in rows if r['replay']),max_calculation_s=max((r['calculation_s'] for r in rows),default=None),
        limitation='Recorded poses/footprints, no executor or closed-loop vehicle; not proof of collision avoidance')
    (args.output/'summary.json').write_text(json.dumps(clean(summary),indent=2)+'\n');print(summary)


if __name__=='__main__':main()
