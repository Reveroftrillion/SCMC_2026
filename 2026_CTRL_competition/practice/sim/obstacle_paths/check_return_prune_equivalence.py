import importlib.util,sys,random,dataclasses,time
from foscar_planning import tunnel_slalom as new
spec=importlib.util.spec_from_file_location('old_slalom',str(__import__('pathlib').Path(__file__).with_name('tunnel_slalom_before_return_prune.py')))
old=importlib.util.module_from_spec(spec);sys.modules[spec.name]=old;spec.loader.exec_module(old)
@dataclasses.dataclass
class Box:
 s_min:float;s_max:float;d_min:float;d_max:float
rng=random.Random(80)
examples=[]
for i in range(50):
 n=1+i%3
 groups=[[Box(15.+j*13.,16.+j*13.,-.5,.5)] for j in range(n)]
 offsets=[-2.5,-2.,-1.5,1.5,2.,2.5]
 if i%7==0:offsets=[-2.,-2.+1e-9,2.,2.+1e-9]
 rng.shuffle(offsets)
 side=i%4
 segment=lambda d0,d1,st,ln,sa,sb,side=side: not (d1==0. and ((side==1 and d0>0.) or (side==2 and d0<0.) or side==3))
 kwargs=dict(groups=groups,feasible=lambda g,offsets=offsets:offsets,s_now=0.,d_now=0.,v=3.+i%3,
             clear_s=lambda b:b.s_max+1.,front=2.,rear=1.,half=.5,min_curve=4.,curve_time=2.,min_move=2.,
             segment_ok=segment,return_length=lambda d:8.+2.*abs(d),max_curvature=.12)
 if i%9==0:kwargs['d_now']=1.5
 examples.append(kwargs)
for label,module in [('baseline',old),('optimized',new)]:
 t=time.perf_counter();results=[dataclasses.asdict(module.plan(**kwargs)) for kwargs in examples]
 print(label,len(results),round((time.perf_counter()-t)*1000,1),'ms')
 if label=='baseline':before=results
 else:
  for i,(a,b) in enumerate(zip(before,results)):
   assert a==b,(i,a,b)
print('All 50 plans exactly equal, including blocked returns and rounded-cost ties.')
