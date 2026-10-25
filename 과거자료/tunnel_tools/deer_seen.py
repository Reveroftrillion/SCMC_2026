"""Deer (truth 13) first seen / first caution / first PEDESTRIAN: ego s, speed, distance; deer start of motion.
python3 deer_seen.py RUN_DIR"""
import os,sys,glob,json,math,bisect
import numpy as np,rosbag2_py
from datetime import datetime
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from rosidl_runtime_py.utilities import get_message
R=sys.argv[1]
def bag_for(i):
    t0=json.loads(open(f'{R}/objects_{i}.jsonl').readline())['t'];best=None
    for b in glob.glob(os.path.expanduser('~/foscar_logs/bags/run_*')):
        try: tb=datetime.strptime(os.path.basename(b)[4:19],'%Y%m%d_%H%M%S').timestamp()
        except ValueError: continue
        if -5<=t0-tb<=300 and (best is None or t0-tb<best[0]): best=(t0-tb,b)
    return best[1]
for i in range(20):
    if not os.path.exists(f'{R}/objects_{i}.jsonl'): continue
    L=[json.loads(x) for x in open(f'{R}/objects_{i}.jsonl')]; ot=[o['t'] for o in L]
    T=[json.loads(x) for x in open(f'{R}/truth_{i}.jsonl')]; tt=[x['t'] for x in T]
    def deer(t): o=L[min(bisect.bisect(ot,t),len(L)-1)]; return next(v for v in o['objects'] if v['id']==13)
    def ego(t): return T[min(bisect.bisect(tt,t),len(T)-1)]
    prev=None;move=None
    for o in L:
        v=next(x for x in o['objects'] if x['id']==13)
        if prev and math.hypot(v['x']-prev[0],v['y']-prev[1])>.05: move=o['t'];break
        prev=(v['x'],v['y'])
    b=bag_for(i);r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=b,storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
    Mt=get_message('foscar_interfaces/msg/DrivingTarget')
    r.set_filter(rosbag2_py.StorageFilter(topics=['/perception/footprints','/control/telemetry','/localization/s']))
    s=0;v_=0;ev={};
    while r.has_next():
        tp,raw,t=r.read_next();m=deserialize_message(raw,String) if tp!='/localization/s' else None
        if tp=='/localization/s': s=deserialize_message(raw,get_message('std_msgs/msg/Float64')).data;continue
        d=json.loads(m.data)
        if tp=='/control/telemetry': v_=d['speed_mps']*3.6;continue
        dv=deer(d['stamp']);e=ego(d['stamp'])
        for f in d['objects']:
            c=f['center']
            if math.hypot(c[0]-dv['x'],c[1]-dv['y'])<1.5:
                dist=math.hypot(dv['x']-e['x'],dv['y']-e['y'])
                for k,cond in (('seen',True),('caution',f.get('caution')),('PED',f['kind']=='PEDESTRIAN')):
                    if cond and k not in ev: ev[k]=(round(s,1),round(v_),round(dist,1),round(d['stamp']-(move or 0),2))
    print('%d seen %s | caution %s | PED %s   (s, km/h, dist m, t-deer move s)'%(i,ev.get('seen'),ev.get('caution'),ev.get('PED')))
