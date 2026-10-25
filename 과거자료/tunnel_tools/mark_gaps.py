"""Deer (truth 13) scan by scan while ahead (practice only): track id, kind, caution; the gaps in the mark and whether
the same track was still there during them. python3 mark_gaps.py RUN_DIR..."""
import os,sys,glob,json,math,bisect
import numpy as np,rosbag2_py
from datetime import datetime
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
def bag_for(R,i):
    t0=json.loads(open(f'{R}/objects_{i}.jsonl').readline())['t'];best=None
    for b in glob.glob(os.path.expanduser('~/foscar_logs/bags/run_*')):
        try: tb=datetime.strptime(os.path.basename(b)[4:19],'%Y%m%d_%H%M%S').timestamp()
        except ValueError: continue
        if -5<=t0-tb<=300 and (best is None or t0-tb<best[0]): best=(t0-tb,b)
    return best[1]
allgaps=[];kept=[0,0]
for R in sys.argv[1:]:
  for i in range(20):
    if not os.path.exists(f'{R}/objects_{i}.jsonl'): continue
    L=[json.loads(x) for x in open(f'{R}/objects_{i}.jsonl')];ot=[o['t'] for o in L]
    T=[json.loads(x) for x in open(f'{R}/truth_{i}.jsonl')];tt=[x['t'] for x in T]
    move=None;prev=None
    for o in L:
        v=next(x for x in o['objects'] if x['id']==13)
        if prev and math.hypot(v['x']-prev[0],v['y']-prev[1])>.05: move=o['t'];break
        prev=(v['x'],v['y'])
    r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=bag_for(R,i),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/perception/footprints']))
    seq=[]
    while r.has_next():
        _,raw,_=r.read_next();d=json.loads(deserialize_message(raw,String).data)
        if move and d['stamp']>move: break
        o=L[min(bisect.bisect(ot,d['stamp']),len(L)-1)];v=next(x for x in o['objects'] if x['id']==13)
        fs=[f for f in d['objects'] if f.get('xy') and min(math.hypot(x-v['x'],y-v['y']) for x,y in f['xy'])<1.5]
        seq.append((d['stamp'],set(f['id'] for f in fs),any(f.get('caution') or f['kind']=='PEDESTRIAN' for f in fs)))
    # gaps in the mark after it was first set
    first=next((k for k,s in enumerate(seq) if s[2]),None)
    if first is None: continue
    k=first;marked_ids=set()
    while k<len(seq):
        if seq[k][2]: marked_ids|=seq[k][1];k+=1;continue
        j=k
        while j<len(seq) and not seq[j][2]: j+=1
        if j<len(seq):
            dur=seq[j][0]-seq[k-1][0];allgaps.append(dur)
            same=all(seq[m][1]&marked_ids for m in range(k,j))
            kept[0 if same else 1]+=1
            print('%s/%d gap %.2f s (%d scans) same track seen throughout: %s'%(os.path.basename(R),i,dur,j-k,same))
        k=j
print('gaps',len(allgaps),'max %.2f s'%max(allgaps) if allgaps else '', 'same track throughout',kept[0],'not',kept[1])
