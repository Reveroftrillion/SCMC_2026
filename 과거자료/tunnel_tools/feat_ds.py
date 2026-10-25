"""Tunnel track features (FEAT log lines) labelled by truth: per class and range band (practice only).
python3 feat_ds.py RUN_DIR... -> writes feat_ds.jsonl next to this script and prints a summary"""
import sys,os,re,json,math,bisect
import numpy as np
NAMES={13:'deer',59:'person',15:'car',16:'barricade',26:'cone',27:'cone',32:'cone',33:'cone'}
pat=re.compile(r'FEAT t=([\d.]+) id=(\d+) mx=([-\d.]+) my=([-\d.]+) x=([-\d.]+) y=([-\d.]+) top=([-\d.]+) long=([\d.]+) short=([\d.]+) spread=([\d.]+) mid=([\d.]+) imed=(\S+) n=(\d+) sx=([\d.]+) sy=([\d.]+) struct=(\d) conf=(\d)(?: pitch=([-\d.]+) plane=(\d))?')
rows=[]
for R in sys.argv[1:]:
  for i in range(20):
    lp=f'{R}/launch_{i}.log'
    if not os.path.exists(lp): continue
    L=[json.loads(x) for x in open(f'{R}/objects_{i}.jsonl')];ot=[o['t'] for o in L]
    for l in open(lp,errors='ignore'):
        m=pat.search(l)
        if not m: continue
        v=m.groups();t=float(v[0]);mx,my=float(v[2]),float(v[3])
        objs=L[min(bisect.bisect(ot,t),len(L)-1)]['objects'];lab='other';best=9
        for o in objs:
            if o['id'] not in NAMES: continue
            dd=math.hypot(o['x']-mx,o['y']-my)-max(o['l'],o['w'])/2
            if dd<.6 and dd<best: best=dd;lab=NAMES[o['id']]
        rows.append(dict(run=f'{os.path.basename(R)}/{i}',id=int(v[1]),lab=lab,x=float(v[4]),y=float(v[5]),top=float(v[6]),
            long=float(v[7]),short=float(v[8]),spread=float(v[9]),mid=float(v[10]),imed=None if v[11]=='None' else float(v[11]),
            n=int(v[12]),sx=float(v[13]),sy=float(v[14]),struct=int(v[15]),conf=int(v[16]),pitch=float(v[17]) if v[17] else None,plane=int(v[18]) if v[18] else None,t=t))
json.dump(rows,open(os.path.join(os.path.dirname(os.path.abspath(__file__)),'feat_ds.json'),'w'))
print(len(rows),'scans')
bands=[(5,15),(15,20),(20,25),(25,30),(30,35),(35,41)]
for lab in ('person','deer','cone','barricade','car','other'):
    print('==',lab)
    for a,b in bands:
        r=[x for x in rows if x['lab']==lab and a<=x['x']<b and abs(x['y'])<12]
        if not r: continue
        top=np.array([x['top'] for x in r]);im=np.array([x['imed'] for x in r if x['imed'] is not None]);w=np.array([max(x['long'],x['short']) for x in r])
        print('  %2d-%2d m n=%4d top p5/50/95 %.2f/%.2f/%.2f | width p50/95 %.2f/%.2f | int p5/50/95 %s'%(a,b,len(r),*np.percentile(top,[5,50,95]),*np.percentile(w,[50,95]),
              '/'.join('%.0f'%q for q in np.percentile(im,[5,50,95])) if len(im) else '-'))
