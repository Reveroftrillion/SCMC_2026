import sys,json,math
from pathlib import Path
import numpy as np
from rosbag2_py import SequentialReader,StorageOptions,ConverterOptions,StorageFilter
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from foscar_control.core import Route
from foscar_planning.frenet import to_frenet
from foscar_planning.avoid_planner import polygon_box
from foscar_planning.tunnel_slalom import sweep_hits
r=SequentialReader();r.open(StorageOptions(uri=sys.argv[1],storage_id=''),ConverterOptions('',''))
types={t.name:get_message(t.type) for t in r.get_all_topics_and_types() if t.name in ['/perception/footprints','/localization/s','/localization/odometry']};r.set_filter(StorageFilter(topics=list(types)))
route=Route(np.loadtxt('src/foscar_bringup/routes/global_path.txt')[:,:2]);s=0;ego=None;sample=None;history={}
while r.has_next():
 topic,b,t=r.read_next();m=deserialize_message(b,types[topic])
 if topic=='/localization/s':s=m.data
 elif topic=='/localization/odometry':ego=m.pose.pose.position
 elif 285.<s<309.8 and ego is not None:
  data=json.loads(m.data);offset=to_frenet(route,ego.x,ego.y,s)[0]-s
  shapes=[]
  for o in data['objects']:
   points=[to_frenet(route,x,y,s+offset) for x,y in o['xy']]
   points=[(ss-offset,dd) for ss,dd in points]
   if points and 315<min(p[0] for p in points)<335 and max(p[1] for p in points)<7 and min(p[1] for p in points)>-7:
    history.setdefault(o['id'],[]).extend(points)
    shapes.append(dict(id=o['id'],points=points,kind=o['kind']))
  if shapes:sample=dict(s=s,offset=offset,shapes=shapes)
sample['history']={o['id']:history[o['id']] for o in sample['shapes']}
Path('practice/sim/obstacle_paths/static_polygon_snapshot.json').write_text(json.dumps(sample,indent=2))
boxes=[polygon_box(sample['history'][o['id']],.2) for o in sample['shapes']]
print('snapshot',sample['s'],sample['offset'],[(o['id'],o['kind']) for o in sample['shapes']])
print('boxes',boxes)
for d in [5.5,5.75,6.]:
 for length in [13.,13.5,14.,14.5,15.,16.,17.,18.]:
  if not sweep_hits(boxes,0.,d,sample['s'],length,sample['s'],340.,3.8,.835,1.246):print('clear',d,length,'k_bound',5.77*d/length**2)
