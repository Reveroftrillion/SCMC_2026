#!/usr/bin/env python3
"""Read an ended course audit bag; create new derived analysis and camera video."""
import argparse
from collections import Counter, defaultdict
import csv
import json
import math
from pathlib import Path
import sys

import rosbag
import numpy as np


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run',required=True,type=Path)
    ap.add_argument('--output',required=True,type=Path)
    args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    rows=[];last={};counts=Counter();states=Counter();signals=Counter();modes=Counter()
    gaps=[];previous=None;max_gap=0.;max_age_error=None;camera=[];debug=[];positions=[];target_rows=[];marker_rows=[]
    selected=['/behavior/target','/behavior/markers','/perception/footprints','/perception/clearance_queries',
              '/behavior/signal_stop','/perception/traffic_signal','/control/telemetry','/localization/odometry',
              '/sensors/camera/traffic/compressed','/perception/traffic_light/debug/compressed','/localization/mode']
    start=end=None
    for path in sorted(args.run.glob('*.bag')):
        with rosbag.Bag(str(path)) as bag:
            for topic,info in bag.get_type_and_topic_info().topics.items():counts[topic]+=info.message_count
            start=min(start or bag.get_start_time(),bag.get_start_time());end=max(end or bag.get_end_time(),bag.get_end_time())
            for topic,msg,t in bag.read_messages(topics=selected):
                now=t.to_sec();item=dict(t=now,topic=topic)
                if topic.endswith('/compressed'):
                    (debug if '/debug/' in topic else camera).append(now);continue
                if topic=='/localization/odometry':
                    positions.append(dict(t=now,x=msg.pose.pose.position.x,y=msg.pose.pose.position.y,speed=msg.twist.twist.linear.x));continue
                if topic=='/behavior/target':
                    states[msg.state]+=1
                    if previous is not None:
                        gap=now-previous;max_gap=max(max_gap,gap)
                        if gap>1:gaps.append(dict(start=previous,end=now,gap_s=gap))
                    previous=now
                    item.update(state=msg.state,v_max=msg.v_max,stop_s=msg.stop_s,offset=msg.lat_offset,
                        entry_s=msg.entry_s,entry_length=msg.entry_length_m,reference_id=msg.reference_id,
                        reference_points=len(msg.reference_x),chain=list(zip(msg.chain_offset,msg.chain_start_s,msg.chain_length_m)))
                    target_rows.append(item)
                elif topic=='/behavior/markers':
                    item['texts']=[dict(ns=m.ns,text=m.text) for m in msg.markers if m.text and m.ns in ('avoid_choice_text','bev_stop_reason')]
                    marker_rows.append(item)
                elif topic=='/localization/mode':modes[msg.data]+=1;continue
                else:
                    try:item['data']=json.loads(msg.data)
                    except Exception:continue
                    if topic=='/perception/traffic_signal':signals[item['data']['id']+':'+item['data']['color']]+=1
                    if topic=='/behavior/signal_stop':
                        v=item['data']
                        if v.get('last_seen') is not None and v.get('age_s') is not None:
                            error=now-v['last_seen']-v['age_s']
                            if max_age_error is None or error>max_age_error['additional_age_s']:
                                max_age_error=dict(t=now,additional_age_s=error,**v)
                if now>=end-12:rows.append(item)
    for gap in gaps:
        gap.update(raw_frames=sum(gap['start']<=v<=gap['end'] for v in camera),debug_frames=sum(gap['start']<=v<=gap['end'] for v in debug))
    follower=[]
    for directory in (args.run/'home/foscar_logs', args.run/'ros-logs/follower'):
        for p in directory.glob('follow*.csv'):
            with p.open() as stream:
                follower+=list(csv.DictReader(stream))
    valid=[r for r in follower if r['s_m'] and r['x'] and r['state']!='WAITING']
    stops=[];i=0
    while i<len(valid):
        if abs(float(valid[i]['speed_mps']))>=.1:i+=1;continue
        j=i+1
        while j<len(valid) and abs(float(valid[j]['speed_mps']))<.1:j+=1
        duration=float(valid[j-1]['elapsed_s'])-float(valid[i]['elapsed_s'])
        if duration>=2:stops.append(dict(start=float(valid[i]['elapsed_s']),duration_s=duration,s=float(valid[i]['s_m']),reasons=dict(Counter(r['reason'] for r in valid[i:j]))))
        i=j
    summary=dict(start=start,end=end,duration_s=end-start,counts=dict(counts),states=dict(states),signals=dict(signals),localization_modes=dict(modes),
        maximum_target_gap_s=max_gap,target_gaps_gt_1s=gaps,largest_additional_signal_age=max_age_error,
        stationary_episodes=stops,follower_reasons=dict(Counter(r['reason'] for r in follower)),
        max_cross_track_m=max((abs(float(r['cross_track_m'])) for r in valid),default=None),
        max_speed_kmh=max((float(r['speed_mps'])*3.6 for r in valid),default=None),
        source_trial=json.loads((args.run/'end.json').read_text()))
    def clean(v):
        if isinstance(v,float) and not math.isfinite(v):return None
        if isinstance(v,dict):return {k:clean(x) for k,x in v.items()}
        if isinstance(v,(tuple,list)):return [clean(x) for x in v]
        return v
    for name,data in [('summary.json',summary),('collision-window.json',rows),('targets.json',target_rows),('markers.json',marker_rows),('positions.json',positions)]:
        (args.output/name).write_text(json.dumps(clean(data),ensure_ascii=False,indent=2)+'\n')
    # All original JPEG messages remain in the bag. This convenient video uses their
    # observed mean rate; timestamps.csv preserves the exact variable timing.
    if len(camera)>1:
        import cv2
        fps=(len(camera)-1)/(camera[-1]-camera[0]);video=None
        with (args.output/'camera-timestamps.csv').open('w') as stamps:
            writer=csv.writer(stamps);writer.writerow(['frame','bag_time','header_time'])
            n=0
            for path in sorted(args.run.glob('*.bag')):
                with rosbag.Bag(str(path)) as bag:
                    for _,msg,t in bag.read_messages(topics=['/sensors/camera/traffic/compressed']):
                        frame=cv2.imdecode(np.frombuffer(msg.data,np.uint8),cv2.IMREAD_COLOR)
                        if frame is None:raise ValueError('undecodable camera frame')
                        if video is None:video=cv2.VideoWriter(str(args.output/'camera.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),fps,(frame.shape[1],frame.shape[0]))
                        if not video.isOpened():raise RuntimeError('video writer failed')
                        video.write(frame);writer.writerow([n,t.to_sec(),msg.header.stamp.to_sec()]);n+=1
            if video:video.release()
        print('video',n,'frames',fps,'fps')
    print(json.dumps(clean({k:v for k,v in summary.items() if k!='source_trial'}),ensure_ascii=False,indent=2))


if __name__=='__main__':main()
