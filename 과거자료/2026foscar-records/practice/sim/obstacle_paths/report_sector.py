"""Summarize section-only runs without changing their raw collision results."""
import json, math, sys
from pathlib import Path
from collections import Counter
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
sys.path.insert(0, str(Path('practice/sim').resolve()))
from loop_runner import box, box_gap, npc_centre, EGO_CENTRE, EGO_L, EGO_W
root=Path('practice/sim/runs')/sys.argv[1]
r=SequentialReader();r.open(StorageOptions(uri='/home/foscar/foscar_logs/bags/run_'+sys.argv[2],storage_id=''),ConverterOptions('',''))
ns=['/behavior/target','/behavior/markers','/localization/s']
types={t.name:get_message(t.type) for t in r.get_all_topics_and_types() if t.name in ns}
r.set_filter(StorageFilter(topics=list(types)))
s=0.;last=None;gaps=[];reasons=Counter();positions=[];prediction=[]
while r.has_next():
 topic,b,t=r.read_next();m=deserialize_message(b,types[topic]);now=t/1e9
 if topic=='/localization/s':s=m.data;positions.append((now,s))
 elif topic=='/behavior/target' and 280<s<370:
  if last is not None:gaps.append((now-last,s))
  last=now
 elif topic=='/behavior/markers' and 310<s<345:
  for mark in m.markers:
   if mark.ns=='bev_stop_reason':
    reasons[mark.text.split('\n')[0]]+=1
    if 'PREDICTED' in mark.text:prediction.append((s,mark.text))
result=json.loads((root/'results.jsonl').read_text());obs=result['obstacles']['2'];ox,oy,oyaw=npc_centre(obs);ob=box(ox,oy,oyaw,obs['l'],obs['w'])
truth=[json.loads(l) for l in (root/'truth_0.jsonl').read_text().splitlines()];tt,ss=np.array(positions).T;clearances=[]
for row in truth:
 s=float(np.interp(row['t'],tt,ss))
 if 310<s<345:
  ego=box(row['x']+EGO_CENTRE*math.cos(row['yaw']),row['y']+EGO_CENTRE*math.sin(row['yaw']),row['yaw'],EGO_L,EGO_W)
  clearances.append((box_gap(ego,ob),s))
out={'run':sys.argv[1],'max_target_gap':max(gaps),'target_gaps_over_half_second':[x for x in gaps if x[0]>.5], 'reasons_s310_345':dict(reasons),'minimum_truth_separating_gap':min(clearances),'predicted_samples':prediction,'result':result['result'],'full_stops':result['full_stops']}
(root/'bev_diagnosis.json').write_text(json.dumps(out,indent=2));print(json.dumps({k:v for k,v in out.items() if k!='predicted_samples'},indent=2))
