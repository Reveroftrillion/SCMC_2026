"""Practice-only comparison of recorded truth bodies and BEV guard inflation."""
import json, math, sys, copy
from pathlib import Path
import numpy as np, yaml
from rosbag2_py import SequentialReader,StorageOptions,ConverterOptions,StorageFilter
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from foscar_control.core import Route
from foscar_planning.state import Context,Target
from foscar_planning.states.avoid import Avoid
from foscar_planning.bev_avoidance import BevAvoidance
from foscar_planning.frenet import to_frenet
sys.path.insert(0,str(Path('practice/sim').resolve()))
from loop_runner import box,npc_centre,EGO_CENTRE,EGO_L,EGO_W

def gap(a,b):
 a,b=np.asarray(a),np.asarray(b)
 edges=np.r_[np.roll(a,-1,axis=0)-a,np.roll(b,-1,axis=0)-b]; axes=np.c_[-edges[:,1],edges[:,0]]
 aa,bb=a@axes.T,b@axes.T
 if not np.any((aa.max(0)<bb.min(0))|(bb.max(0)<aa.min(0))):return 0.
 def pd(p,q):
  d=np.roll(q,-1,axis=0)-q
  v=p[:,None,:]-q
  u=np.clip((v*d).sum(2)/(d*d).sum(1),0,1)
  return np.sqrt(((v-u[:,:,None]*d)**2).sum(2)).min()
 return float(min(pd(a,b),pd(b,a)))

root=Path('practice/sim/runs/20261006_174505')
truth=[json.loads(l) for l in (root/'truth_0.jsonl').read_text().splitlines()]
objects=[json.loads(l) for l in (root/'objects_0.jsonl').read_text().splitlines()]
obj=next(o for o in objects[-1]['objects'] if o['id']==2)
ox,oy,oyaw=npc_centre(obj); obstacle=box(ox,oy,oyaw,obj['l'],obj['w'])
route=Route(np.loadtxt('src/foscar_bringup/routes/global_path.txt')[:,:2])
config=yaml.safe_load(Path('src/foscar_planning/config/behavior.yaml').read_text())
params=next(iter(config.values()))['ros__parameters']
a=Avoid(**{k[6:]:v for k,v in params.items() if k.startswith('avoid.')})
bev=BevAvoidance(a.d_step_m,a.horizon_m)
r=SequentialReader();r.open(StorageOptions(uri='/home/foscar/foscar_logs/bags/run_20261006_174510',storage_id=''),ConverterOptions('',''))
types={t.name:get_message(t.type) for t in r.get_all_topics_and_types() if t.name in ['/perception/footprints','/localization/odometry','/localization/s','/behavior/target','/perception/traffic_signal']};r.set_filter(StorageFilter(topics=list(types)))
s=0.;odo=None;scan=None;signal='';samples={};rows=[]
locations=[296.2,297.7,298.6,300.1,303.2,309.3,309.7,311.4]
while r.has_next():
 topic,b,t=r.read_next();m=deserialize_message(b,types[topic]);now=t/1e9
 if topic=='/localization/s':s=m.data;continue
 if topic=='/localization/odometry':odo=m;continue
 if topic=='/perception/traffic_signal':signal=m.data;continue
 if topic=='/perception/footprints':scan=json.loads(m.data);continue
 if not (270<s<400) or odo is None or scan is None:continue
 pos=odo.pose.pose.position;q=odo.pose.pose.orientation;yaw=2*math.atan2(q.z,q.w)
 v=odo.twist.twist.linear.x
 ctx=Context(s,pos.x,pos.y,yaw,v,now,route=route,mission_name='STATIC_OBSTACLE')
 ctx.s_offset=to_frenet(route,pos.x,pos.y,s)[0]-s
 ctx.occupancy_stamp=scan['stamp'];ctx.occupancy_hulls={o['id']:o['xy'] for o in scan['objects'] if len(o['xy'])>=3};ctx.occupancy_sensor=(pos.x,pos.y)
 bev.update(ctx)
 real=min(truth,key=lambda x:abs(x['t']-now));ego=box(real['x']+EGO_CENTRE*math.cos(real['yaw']),real['y']+EGO_CENTRE*math.sin(real['yaw']),real['yaw'],EGO_L,EGO_W)
 row=dict(t=now,s=s,kmh=v*3.6,gap_m=gap(ego,obstacle),target_kmh=m.v_max*3.6,stop_s=m.stop_s,state=m.state,signal=signal,chain=list(zip(m.chain_offset,m.chain_start_s,m.chain_length_m)),ego=ego,regions=[dict(ready=x.ready,velocity=x.velocity,sigma=x.sigma,polygon=x.polygon,stamp=x.stamp) for x in bev.motion.regions])
 rows.append(row)
 for loc in locations:
  if abs(s-loc)<.13:
   score=abs(v)+abs(s-loc)*.05
   if loc not in samples or score<samples[loc][0]:samples[loc]=(score,copy.deepcopy(ctx),copy.deepcopy(bev),row)

out=[]
for loc,(_,ctx,b,row) in samples.items():
 # Counterfactual guard-only replay; recorded target already contains guard/signal caps.
 # Keep recorded path, use 60km/h input cap to inspect geometric stopping decision.
 a.slalom=None
 chain=tuple(row['chain']); target=Target(v_max=60/3.6,chain=chain,lat_offset=0.)
 detail={}
 def trace(frame,event,arg):
  if frame.f_code.co_name=='guard' and event=='return':
   f=frame.f_locals
   for k in ('s','distance','free','arrival','prediction_time','latest','limits','reason'):
    if k in f:detail[k]=f[k]
   if f.get('reason')=='predicted occupancy':
    reg=f['region'];detail['blocking_region']=dict(ready=reg.ready,velocity=reg.velocity,sigma=reg.sigma,polygon=reg.polygon)
    detail['predicted_polygon']=f['p'].tolist();detail['checked_body']=f['body']
  return trace
 sys.settrace(trace)
 try:result=b.guard(ctx,target,a)
 finally:sys.settrace(None)
 row['guard_replay']=dict(detail,cap_kmh=result.v_max*3.6,stop_s=result.stop_s,last_reason=b.last_reason)
 row['truth_gap_at_checked_body_m']=gap(detail['checked_body'],obstacle) if 'checked_body' in detail else None
 if chain:
  clearances=[]
  for ss in np.arange(ctx.s,345,.05):
   d=0.;prev=0.;slope=0.
   for off,start,length in chain:
    if ss>=start:
     u=np.clip((ss-start)/max(length,1e-6),0,1);d=prev+(off-prev)*u**3*(10-15*u+6*u*u);slope=(off-prev)*30*u*u*(1-u)**2/max(length,1e-6)
    prev=off
   body=b.body(ctx,ss,d,slope,a.front_m,a.rear_overhang_m,a.car_width_m/2)
   clearances.append((gap(body,obstacle),ss))
  row['planned_truth_min_gap_m'],row['planned_truth_min_at_s']=min(clearances)
 out.append(row)
report=dict(run=str(root),method='Truth oriented-body Euclidean gap, runner rear-origin conventions. Motion replay at recorded target ticks uses latest odometry as sensor origin; diagnostic guard input cap=60km/h, not exact pre-guard target. No driving code changes.',fixed=dict(car_width=a.car_width_m,side_margin=a.margin_m,grid=b.field.resolution,front=a.front_m,rear=a.rear_overhang_m),object=obj,truth_polygon=obstacle,samples=out)
Path('practice/sim/obstacle_paths/bev_clearance.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
for row in out:
 print(json.dumps({k:row[k] for k in ['s','kmh','gap_m','state','target_kmh','stop_s','planned_truth_min_gap_m','planned_truth_min_at_s'] if k in row}))
 print('guard',row['guard_replay'])
print('fixed',report['fixed'])
