"""Compare observed map hulls with recorded first-deer positions, offline only."""
import json,sqlite3,sys
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
run=Path('practice/sim/runs')/sys.argv[1]
bag=Path('/home/foscar/foscar_logs/bags')/('run_'+sys.argv[2])
rows=[json.loads(l) for l in (run/'objects_0.jsonl').read_text().splitlines()]
times=np.array([r['t'] for r in rows]);initial=next(o for o in rows[0]['objects'] if o['id']==13)
c=sqlite3.connect(str(next(bag.glob('*.db3'))))
topics={i:(n,get_message(t)) for i,n,t in c.execute('select id,name,type from topics') if n in ('/localization/s','/perception/footprints')}
s=None;matches=[]
for i,t,b in c.execute('select topic_id,timestamp,data from messages order by timestamp'):
 if i not in topics:continue
 n,cls=topics[i];m=deserialize_message(b,cls)
 if n=='/localization/s':s=m.data;continue
 data=json.loads(m.data);stamp=data['stamp'];idx=int(np.argmin(abs(times-stamp)));truth=next(o for o in rows[idx]['objects'] if o['id']==13)
 for obj in data['objects']:
  centre=np.mean(obj['xy'],axis=0);distance=float(np.linalg.norm(centre-[truth['x'],truth['y']]))
  if distance<1.5:
   matches.append(dict(stamp=stamp,s=s,id=obj['id'],centroid_distance_m=distance,kind=obj['kind'],
     displacement_from_spawn_m=float(np.hypot(truth['x']-initial['x'],truth['y']-initial['y'])),
     sync_error_s=float(abs(times[idx]-stamp))))
first=matches[0] if matches else None
out=dict(first=first,matched_frames=len(matches),stationary_spawn_frames=sum(m['displacement_from_spawn_m']<.1 for m in matches),
 note='Approximate nearest map centroid within 1.5 m of recorded ID 13; movement measured from spawn position.')
(run/'first_deer_occupancy.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
