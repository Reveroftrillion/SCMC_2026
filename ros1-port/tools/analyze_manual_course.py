#!/usr/bin/env python3
"""Read-only manual-course bag audit; no ROS master or vehicle control.

Checkpoint evidence uses rear-axle odometry, gated by route station and order.
Collision packets and post-recorder handoff are not implied by bag completion.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import numpy as np
import rosbag
from ctrl_control.ctrl_core import Route
from ctrl_control.ctrl_mission_config import read_checkpoints


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    port=Path(__file__).resolve().parents[1]
    route=Route(np.loadtxt(port/'src/ctrl_bringup/routes/global_path.txt'))
    checkpoints=read_checkpoints(port/'src/ctrl_bringup/routes/checkpoints.txt')
    cps=[dict(name='START',xy=route.points[0],radius=3.,station=0.)]
    cps += [dict(name='CP'+str(k),xy=v['xy'],radius=v['radius'],station=route.project(*v['xy'])[0]) for k,v in checkpoints.items()]
    cps.sort(key=lambda p:p['station'])
    cps.append(dict(name='END',xy=route.points[-1],radius=3.,station=float(route.s[-1])))
    for i,p in enumerate(cps):
        p['lo']=-math.inf if i==0 else (cps[i-1]['station']+p['station'])/2
        p['hi']=math.inf if i==len(cps)-1 else (p['station']+cps[i+1]['station'])/2
        p['minimum_distance_m']=math.inf;p['first_inside']=None
    last_s=None;mission=None;states=Counter();transitions=[];modes=[];last_mode=None
    target_previous=None;gaps=[];max_gap=0.;completed=[];last_status=None;last_control=None
    static_offsets=[];static_actual=[];counts=Counter()
    topics=['/localization/s','/localization/odometry','/behavior/mission','/behavior/target',
            '/vehicle/status','/control/state','/localization/mode']
    with rosbag.Bag(str(args.bag)) as bag:
        duration=bag.get_end_time()-bag.get_start_time()
        all_topics=bag.get_type_and_topic_info().topics
        for topic,msg,t in bag.read_messages(topics=topics):
            now=t.to_sec();counts[topic]+=1
            if topic=='/localization/s':last_s=msg.data
            elif topic=='/localization/odometry' and last_s is not None:
                p=msg.pose.pose.position;xy=np.array([p.x,p.y])
                for cp in cps:
                    if cp['lo']<=last_s<=cp['hi']:
                        dist=float(np.linalg.norm(xy-cp['xy']))
                        cp['minimum_distance_m']=min(cp['minimum_distance_m'],dist)
                        if dist<=cp['radius'] and cp['first_inside'] is None:cp['first_inside']=now
                if mission=='STATIC_OBSTACLE':
                    s,_,h=route.project(*xy,previous=last_s);q=route.at(s)
                    static_actual.append(float(-(p.x-q[0])*math.sin(h)+(p.y-q[1])*math.cos(h)))
            elif topic=='/behavior/mission':
                d=json.loads(msg.data);name=d['mission'];states[name+':'+d.get('substate','')]+=1
                if name!=mission:transitions.append(dict(t=now,s=last_s,mission=name));mission=name
                if d.get('pedestrian_yield_completed') and name=='PEDESTRIAN':completed.append(dict(t=now,s=last_s))
            elif topic=='/behavior/target':
                if target_previous is not None:
                    gap=now-target_previous;max_gap=max(max_gap,gap)
                    if gap>1:gaps.append(dict(t=now,s=last_s,gap=gap))
                target_previous=now
                if mission=='STATIC_OBSTACLE':static_offsets.append(msg.lat_offset)
            elif topic=='/localization/mode':
                if msg.data!=last_mode:modes.append(dict(t=now,s=last_s,mode=msg.data));last_mode=msg.data
            elif topic=='/vehicle/status':
                last_status=dict(t=now,**{k:getattr(msg,k) for k in msg.__slots__ if k!='header'})
            elif topic=='/control/state':last_control=dict(t=now,value=msg.data)
    for cp in cps:
        cp['xy']=cp['xy'].tolist();cp.pop('lo');cp.pop('hi')
        if not math.isfinite(cp['minimum_distance_m']):cp['minimum_distance_m']=None
    passed=[p['name'] for p in sorted([p for p in cps if p['first_inside'] is not None],key=lambda p:p['first_inside'])]
    expected=[p['name'] for p in cps]
    summary=dict(bag=str(args.bag),bag_sha256=hashlib.sha256(args.bag.read_bytes()).hexdigest(),duration_s=duration,
        checkpoints=cps,passed_order=passed,all_checkpoints_in_order=passed==expected,counts=dict(counts),
        all_recorded_topics=sorted(all_topics),mission_states=dict(states),transitions=transitions,
        localization_transitions=modes,maximum_target_gap_s=max_gap,target_gaps_gt_1s=gaps,
        static_requested_offset_range=[min(static_offsets),max(static_offsets)] if static_offsets else None,
        static_actual_offset_range=[min(static_actual),max(static_actual)] if static_actual else None,
        pedestrian_completed_first=completed[0] if completed else None,last_status=last_status,last_control=last_control,
        limitations=['Rear-axle local odometry, not official scoring',
                    'No raw CollisionData supervision in this manual bag; no collision-free claim',
                    'Recorder may stop before bridge handoff; Manual/P requires separate feedback'])
    (args.output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:summary[k] for k in ['duration_s','all_checkpoints_in_order','passed_order','maximum_target_gap_s','static_requested_offset_range','static_actual_offset_range','pedestrian_completed_first','last_status','last_control']},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
