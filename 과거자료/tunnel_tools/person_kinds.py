"""Person (truth 59) scan by scan before it moves: footprint ids and kinds near it (practice only)."""
import os,sys,glob,json,math,bisect
import rosbag2_py
from datetime import datetime
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
R=sys.argv[1]
def bag_for(i):
    t0=json.loads(open(f'{R}/objects_{i}.jsonl').readline())['t'];best=None
    for b in glob.glob(os.path.expanduser('~/foscar_logs/bags/run_*')):
        try: tb=datetime.strptime(os.path.basename(b)[4:19],'%Y%m%d_%H%M%S').timestamp()
        except ValueError: continue
        if -5<=t0-tb<=300 and (best is None or t0-tb<best[0]): best=(t0-tb,b)
    return best[1]
for i in [int(a) for a in sys.argv[2:]]:
    L=[json.loads(x) for x in open(f'{R}/objects_{i}.jsonl')];ot=[o['t'] for o in L]
    T=[json.loads(x) for x in open(f'{R}/truth_{i}.jsonl')];tt=[x['t'] for x in T]
    move=None;prev=None
    for o in L:
        v=next(x for x in o['objects'] if x['id']==59)
        if prev and math.hypot(v['x']-prev[0],v['y']-prev[1])>.05: move=o['t'];break
        prev=(v['x'],v['y'])
    r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=bag_for(i),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/perception/footprints']))
    out=[]
    while r.has_next():
        _,raw,_=r.read_next();d=json.loads(deserialize_message(raw,String).data)
        if move and d['stamp']>move+.3: break
        o=L[min(bisect.bisect(ot,d['stamp']),len(L)-1)];v=next(x for x in o['objects'] if x['id']==59)
        e=T[min(bisect.bisect(tt,d['stamp']),len(T)-1)];dist=math.hypot(v['x']-e['x'],v['y']-e['y'])
        if dist>36: continue
        fs=[f for f in d['objects'] if f.get('xy') and min(math.hypot(x-v['x'],y-v['y']) for x,y in f['xy'])<1.]
        out.append('%.0f:%s'%(dist,','.join('%s%s'%(f['id'][-3:],f['kind'][0]) for f in fs) or '-'))
    print(i,' '.join(out))
