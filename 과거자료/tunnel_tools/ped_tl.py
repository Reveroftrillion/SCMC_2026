"""Yield timeline vs truth (practice only): times rel. to YIELD on of stop, body edge in/out of our 1.3 m band,
release; and the earliest 'predictive' release (moving away, at our arrival outside the band). python3 ped_tl.py RUN_DIR"""
import sys,os,json,math,re,bisect
import numpy as np
sys.path.insert(0,os.path.dirname(os.path.abspath(__file__)))
exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)),'release_timing.py')).read().split('R=sys.argv[1]')[0])
R=sys.argv[1]
for i in range(5):
    if not os.path.exists(f'{R}/objects_{i}.jsonl'): continue
    log=open(f'{R}/launch_{i}.log',errors='ignore').read()
    tr=[(float(a),b) for a,b in re.findall(r'\[(\d+\.\d+)\] \[behavior_node\]: \S+ → (\S+)',log)]
    ons=[];st=''
    for t,s_ in tr:
        y='YIELD' in s_.split('|')[0]
        if y and not st: ons.append([t,None])
        if not y and st: ons[-1][1]=t
        st=y
    objs=[json.loads(l) for l in open(f'{R}/objects_{i}.jsonl')]
    ego=[json.loads(l) for l in open(f'{R}/truth_{i}.jsonl')]; et=[x['t'] for x in ego]
    for (on,off),who in zip(ons,(13,59)):
        half=.55 if who==13 else .25
        rows=[]
        for o in objs:
            if not on-1<o['t']<off+3: continue
            ob=next((b for b in o['objects'] if b['id']==who),None)
            if ob is None: continue
            k=min(bisect.bisect(et,o['t']),len(et)-1); es,ed=sd(ego[k]['x'],ego[k]['y'])
            s,d=sd(ob['x'],ob['y']); rows.append((o['t'],s,d-ed,es,ego[k]))
        tt=np.array([r[0] for r in rows]); dd=np.array([r[2] for r in rows])
        rate=np.gradient(dd,tt)
        def first(c,after=on-1):
            return next((r[0]-on for r,cc in zip(rows,c) if cc and r[0]>after),float('nan'))
        edge=np.abs(dd)-half
        inb=first(edge<=1.3); outb=first((edge>1.3)&(dd*rate>0),on); 
        # stopped time
        stop=next((r[0]-on for r,a,b in zip(rows,rows,rows[1:]) if r[0]>on and math.hypot(b[4]['x']-a[4]['x'],b[4]['y']-a[4]['y'])<.01),float('nan'))
        # predictive: at t (car stopped, after stop), gap = ped s - car s - 3.8, arrive with 2 m/s^2 ; body edge at arrive
        pred=float('nan')
        for r,rt in zip(rows,rate):
            if r[0]-on<stop or abs(rt)<1. or r[2]*rt<=0: continue
            gap=max(0.,r[1]-r[3]-3.8); arr=math.sqrt(2*2.*gap)/2.
            if abs(r[2]+rt*arr)-half>1.3: pred=r[0]-on; break
        print('%s/%d %-6s stop %.2f | in band %.2f | out band %.2f | release %.2f | predictive %.2f | speed %.1f m/s gap %.1f m'%(
            os.path.basename(R),i,'deer' if who==13 else 'person',stop,inb,outb,off-on,pred,np.median(np.abs(rate[(tt>on)&(tt<off)])),
            next(r[1]-r[3]-3.8 for r in rows if r[0]-on>=stop)))
