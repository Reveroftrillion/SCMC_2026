"""Per truth object: ego speed and distance at first seen, first 3-scan stable, first caution, first PEDESTRIAN
(practice only). python3 detect_dist.py RUN_DIR..."""
import os,sys,glob,json,math,bisect
import numpy as np,rosbag2_py
from datetime import datetime
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from rosidl_runtime_py.utilities import get_message
OBJ={13:'deer',15:'car',16:'barricade',26:'cone',27:'cone',32:'cone',33:'cone',59:'person'}
def bag_for(R,i):
    t0=json.loads(open(f'{R}/objects_{i}.jsonl').readline())['t'];best=None
    for b in glob.glob(os.path.expanduser('~/foscar_logs/bags/run_*')):
        try: tb=datetime.strptime(os.path.basename(b)[4:19],'%Y%m%d_%H%M%S').timestamp()
        except ValueError: continue
        if -5<=t0-tb<=300 and (best is None or t0-tb<best[0]): best=(t0-tb,b)
    return best[1]
Ff=get_message('std_msgs/msg/Float64')
for R in sys.argv[1:]:
  for i in range(20):
    if not os.path.exists(f'{R}/objects_{i}.jsonl'): continue
    L=[json.loads(x) for x in open(f'{R}/objects_{i}.jsonl')]; ot=[o['t'] for o in L]
    T=[json.loads(x) for x in open(f'{R}/truth_{i}.jsonl')]; tt=[x['t'] for x in T]
    def obj(t): return L[min(bisect.bisect(ot,t),len(L)-1)]['objects']
    def ego(t): return T[min(bisect.bisect(tt,t),len(T)-1)]
    r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=bag_for(R,i),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/perception/footprints','/control/telemetry']))
    v=0;ev={};streak={}
    while r.has_next():
        tp,raw,t=r.read_next();d=json.loads(deserialize_message(raw,String).data)
        if tp=='/control/telemetry': v=d['speed_mps']*3.6;continue
        e=ego(d['stamp']);hit=set()
        for o in obj(d['stamp']):
            if o['id'] not in OBJ: continue
            dist=math.hypot(o['x']-e['x'],o['y']-e['y'])
            fs=[f for f in d['objects'] if f.get('xy') and min(math.hypot(x-o['x'],y-o['y']) for x,y in f['xy'])<max(o['l'],o['w'])/2+.5]
            if not fs: streak[o['id']]=0;continue
            streak[o['id']]=streak.get(o['id'],0)+1
            k=o['id']
            for name,c in (('seen',True),('stable',streak[k]>=3),('caution',any(f.get('caution') for f in fs)),('PED',any(f['kind']=='PEDESTRIAN' for f in fs))):
                if c and (k,name) not in ev: ev[(k,name)]=(round(v),round(dist,1))
    for k in (13,59):
        print('%s/%d %-9s %s'%(os.path.basename(R),i,OBJ[k]+str(k),' '.join('%s %s@%skm/h'%(n,ev[(k,n)][1],ev[(k,n)][0]) for n in ('seen','stable','caution','PED') if (k,n) in ev)))
