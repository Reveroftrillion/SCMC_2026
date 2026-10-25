"""YIELD release vs pedestrian leaving our way (practice only). python3 release_timing.py RUN_DIR"""
import sys,os,json,math,re,bisect,glob
from datetime import datetime
import numpy as np,yaml,rosbag2_py
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from foscar_control.core import Route
from foscar_control.mission_config import geometry_path
p=Path('/home/foscar/Desktop/2026Morai/2026foscar-competition/src/foscar_planning/config/missions.yaml'); g=yaml.safe_load(p.read_text())['geometry']
route=Route(np.loadtxt(geometry_path(g['route'],p.parent)))
def sd(x,y):
    s,dist,h=route.project(x,y); qx,qy=route.at(s); return s,dist*np.sign(math.cos(h)*(y-qy)-math.sin(h)*(x-qx))
def bag_for(R,i):
    t0=json.loads(open(f'{R}/objects_{i}.jsonl').readline())['t']; best=None
    for b in glob.glob(os.path.expanduser('~/foscar_logs/bags/run_*')):
        try: tb=datetime.strptime(os.path.basename(b)[4:19],'%Y%m%d_%H%M%S').timestamp()
        except ValueError: continue
        if -5<=t0-tb<=300 and (best is None or t0-tb<best[0]): best=(t0-tb,b)
    return best[1]
R=sys.argv[1]
for i in range(20):
    if not os.path.exists(f'{R}/objects_{i}.jsonl'): continue
    objs=[json.loads(l) for l in open(f'{R}/objects_{i}.jsonl')]
    b=bag_for(R,i)
    r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=b,storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
    types={x.name:x.type for x in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=['/behavior/state','/localization/s','/control/telemetry','/behavior/target']))
    Mb=get_message(types['/behavior/state']); Ms=get_message(types['/localization/s']); Mt=get_message(types['/control/telemetry'])
    st=None; es=0; ev=[]; cur=None; moved=None
    while r.has_next():
        tp,raw,t=r.read_next(); t*=1e-9
        if tp=='/localization/s': es=deserialize_message(raw,Ms).data; continue
        if tp=='/behavior/state':
            s_=deserialize_message(raw,Mb).data.split('|')[0]
            if s_=='YIELD' and st!='YIELD': cur=dict(on=t,s=es)
            if st=='YIELD' and s_!='YIELD' and cur: cur['off']=t; ev.append(cur); cur=None
            st=s_; continue
        if tp=='/control/telemetry':
            d=json.loads(deserialize_message(raw,Mt).data)
            if ev and 'go' not in ev[-1] and d['accel']>0.05: ev[-1]['go']=t
    for e,who in zip(ev,(13,59)):
        # truth: when the walker's body left a 3.6 m band around our stopped line (d of the car +-(1.0 half + 0.8)) and the lanes
        tr=[(o['t'],*sd(ob['x'],ob['y'])) for o in objs for ob in o['objects'] if ob['id']==who and e['on']-1<o['t']<e['off']+3]
        ego=[json.loads(l) for l in open(f'{R}/truth_{i}.jsonl')]; et=[x['t'] for x in ego]
        k=bisect.bisect(et,e['off']); dc=sd(ego[min(k,len(ego)-1)]['x'],ego[min(k,len(ego)-1)]['y'])[1]
        half=.55 if who==13 else .25
        out_car=next((t for t,s,d in tr if abs(d-dc)-half>1.8 and t>e['on']),None)
        out_lane=next((t for t,s,d in tr if (d-half>5.25 or d+half<-1.75) and t>e['on']),None)
        print('%s/%d %s: stop->release %.2f s | left our line(+-1.8 m) %+.2f s before release | left both lanes %+.2f s | release->accel %.2f s'%(
            os.path.basename(R),i,'deer' if who==13 else 'person',e['off']-e['on'],
            (e['off']-out_car) if out_car else float('nan'),(e['off']-out_lane) if out_lane else float('nan'),
            (e.get('go',e['off'])-e['off'])))
