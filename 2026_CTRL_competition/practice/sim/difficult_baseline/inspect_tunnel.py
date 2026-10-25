"""Practice-only post-run evidence; never consumed by vehicle nodes."""
import json, sqlite3, math, collections
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2
ROOT=Path(__file__).resolve().parents[3]
RUN=ROOT/'practice/sim/runs/20261006_161105'
BAG=Path.home()/'foscar_logs/bags/run_20261006_161109'

def rows(path,names):
 c=sqlite3.connect(str(next(path.glob('*.db3'))))
 ts={i:(n,get_message(t)) for i,n,t in c.execute('select id,name,type from topics') if n in names}
 q='select topic_id,timestamp,data from messages where topic_id in ('+','.join(map(str,ts))+') order by timestamp'
 for tid,t,data in c.execute(q):
  n,cls=ts[tid];yield n,t*1e-9,deserialize_message(data,cls)

objects=[json.loads(l) for l in (RUN/'objects_0.jsonl').read_text().splitlines()]
ot=np.array([o['t'] for o in objects]);pose=None;ego_s=0.;events=[];states=[]
for n,t,m in rows(BAG,{'/localization/odometry','/localization/s','/events','/behavior/target'}):
 if n=='/localization/s':ego_s=m.data
 elif n=='/localization/odometry':
  p=m.pose.pose.position;q=m.pose.pose.orientation;pose=(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
 elif n=='/behavior/target':
  if not states or states[-1]['state']!=m.state:states.append(dict(t=t,s=ego_s,state=m.state))
 elif pose:
  co,si=math.cos(pose[2]),math.sin(pose[2]);x=pose[0]+co*m.x-si*m.y;y=pose[1]+si*m.x+co*m.y
  sample=objects[min(len(objects)-1,int(np.searchsorted(ot,t)))]
  close=[(math.hypot(x-o['x'],y-o['y']),o['id']) for o in sample['objects'] if o['id'] in (13,15,59)]
  dist,oid=min(close,default=(math.inf,-1))
  if dist<2.5:events.append(dict(t=t,s=ego_s,id=oid,dist=dist,kind=m.type,speed=math.hypot(m.vx,m.vy),key=m.value))
counts=collections.Counter((e['id'],e['kind']) for e in events)
frames=[];pose=None
for n,t,m in rows(Path(str(BAG)+'_points'),{'/localization/odometry','/sensors/lidar/points'}):
 if n=='/localization/odometry':
  p=m.pose.pose.position;q=m.pose.pose.orientation;pose=(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)));continue
 if pose is None or t>1791270683:continue
 sample=objects[min(len(objects)-1,int(np.searchsorted(ot,t)))];obj=next((o for o in sample['objects'] if o['id']==13),None)
 if obj is None:continue
 pts=point_cloud2.read_points_numpy(m,field_names=('x','y','z','intensity'),skip_nans=True)
 co,si=math.cos(pose[2]),math.sin(pose[2]);wx=pose[0]+co*(pts[:,0]+2)-si*pts[:,1];wy=pose[1]+si*(pts[:,0]+2)+co*pts[:,1]
 yaw=math.radians(obj['yaw']);dx,dy=wx-obj['x'],wy-obj['y'];ox=math.cos(yaw)*dx+math.sin(yaw)*dy;oy=-math.sin(yaw)*dx+math.cos(yaw)*dy
 sel=(abs(ox)<obj['l']/2+.3)&(abs(oy)<obj['w']/2+.3)&(pts[:,2]+1.75>.25)&(pts[:,2]+1.75<obj['h']+.3)
 p=pts[sel]
 if len(p):frames.append(dict(t=t,n=len(p),zero_fraction=float(np.mean(p[:,3]==0)),range=math.hypot(obj['x']-pose[0],obj['y']-pose[1])))
out=dict(run=str(RUN),states=states,near_truth_event_counts={str(k):v for k,v in counts.items()},first_events={str(oid):next((e for e in events if e['id']==oid),None) for oid in (13,15,59)},object13_points=dict(frames=len(frames),total=sum(f['n'] for f in frames),first_frames=frames[:5]),notes=['Truth association is approximate nearest-center matching, not tracker identity.','Point selection uses yaw-only pose and expanded truth box, includes possible clutter; no claim of deer semantic identity.','No collision packet, but ay=40.06 at s1877: not a clean pass.','Stopped at s1892.8 while static ID15 vicinity remained VEHICLE.'])
(ROOT/'practice/sim/difficult_baseline/tunnel_evidence.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
