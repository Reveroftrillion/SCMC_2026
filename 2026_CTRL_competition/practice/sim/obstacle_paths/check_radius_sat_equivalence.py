import importlib.util,sys,random,dataclasses,time
from foscar_planning import tunnel_slalom as new
spec=importlib.util.spec_from_file_location('old_sat',str(__import__('pathlib').Path(__file__).with_name('tunnel_slalom_before_radius_sat.py')))
old=importlib.util.module_from_spec(spec);sys.modules[spec.name]=old;spec.loader.exec_module(old)
@dataclasses.dataclass
class Box:
 s_min:float;s_max:float;d_min:float;d_max:float
rng=random.Random(80)
cases=[]
for i in range(3000):
 start=rng.uniform(0,2200);length=rng.uniform(.1,40);sa=start+rng.uniform(-5,10);sb=sa+rng.uniform(0,50)
 front=rng.uniform(.1,5);rear=rng.uniform(.1,2);half=rng.uniform(.1,2)
 boxes=[]
 for j in range(1+i%5):
  bs=rng.uniform(sa-5,sb+5);bd=rng.uniform(-8,8)
  boxes.append(Box(bs,bs+rng.uniform(.001,5),bd,bd+rng.uniform(.001,5)))
 cases.append((boxes,rng.uniform(-6,6),rng.uniform(-6,6),start,length,sa,sb,front,rear,half))
for label,module in [('corners',old),('radii',new)]:
 t=time.perf_counter();results=[module.sweep_hits(*case) for case in cases]
 print(label,len(results),round((time.perf_counter()-t)*1000,1),'ms')
 if label=='corners':before=results
 else:
  for i,(a,b) in enumerate(zip(before,results)):assert a==b,(i,cases[i],a,b)
print('All 3000 sweep collision decisions equal.')
