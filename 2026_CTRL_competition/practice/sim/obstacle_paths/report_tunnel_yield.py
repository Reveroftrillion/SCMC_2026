"""Compare YIELD episodes against nearby recorded objects, without driving inputs."""
import json,math,sys
from pathlib import Path
import numpy as np
from rosbag2_py import SequentialReader,StorageOptions,ConverterOptions,StorageFilter
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
root=Path('practice/sim/runs')/sys.argv[1];bag=sys.argv[2]
objects=[json.loads(l) for l in (root/'objects_0.jsonl').read_text().splitlines()];ot=np.array([o['t'] for o in objects])
r=SequentialReader();r.open(StorageOptions(uri='/home/foscar/foscar_logs/bags/run_'+bag,storage_id=''),ConverterOptions('',''))
ns=['/behavior/target','/localization/s','/localization/odometry','/events'];types={x.name:get_message(x.type) for x in r.get_all_topics_and_types() if x.name in ns};r.set_filter(StorageFilter(topics=list(types)))
s=0.;pose=None;events={};episodes=[];active=None;last=None;gaps=[]
while r.has_next():
 topic,b,t=r.read_next();m=deserialize_message(b,types[topic]);now=t/1e9
 if topic=='/localization/s':s=m.data
 elif topic.endswith('odometry'):pose=m
 elif topic=='/events' and pose is not None:
  p=pose.pose.pose.position;q=pose.pose.pose.orientation;h=2*math.atan2(q.z,q.w)
  x=p.x+math.cos(h)*m.x-math.sin(h)*m.y;y=p.y+math.sin(h)*m.x+math.cos(h)*m.y
  sample=objects[min(len(objects)-1,int(np.searchsorted(ot,now)))]['objects'];d,obj=min(((math.hypot(x-o['x'],y-o['y']),o) for o in sample),key=lambda v:v[0])
  events[m.value]=dict(t=now,s=m.s,kind=m.type,size=[m.size_x,m.size_y],nearest_truth=obj['id'],truth_kind=obj['type'],truth_speed=obj['v'],distance=d)
 elif topic.endswith('target'):
  if last is not None:gaps.append((now-last,s))
  last=now
  if m.state=='YIELD':
   if active is None:active=dict(start_s=s,start_t=now,min_cap_kmh=m.v_max*3.6,matches=[]);episodes.append(active)
   active['min_cap_kmh']=min(active['min_cap_kmh'],m.v_max*3.6)
   possible=[e for e in events.values() if now-e['t']<.5 and e['kind'] in ['PEDESTRIAN','DYNAMIC_UNKNOWN','OBSTACLE']]
   if possible:
    e=min(possible,key=lambda e:abs(e['s']-(m.stop_s+6.)))
    if abs(e['s']-(m.stop_s+6.))<2.:
     row={k:e[k] for k in ['kind','nearest_truth','truth_kind','truth_speed','distance','size']}
     if not active['matches'] or active['matches'][-1]['nearest_truth']!=row['nearest_truth']:active['matches'].append(row)
  elif active is not None:
   active.update(end_s=s,end_t=now,duration_s=now-active['start_t']);active=None
out={'episodes':episodes,'max_target_gap':max(gaps),'matching_note':'Nearest object to latest-odometry transformed event, matched to target.stop_s+6; approximate association.'}
(root/'yield_episodes.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
