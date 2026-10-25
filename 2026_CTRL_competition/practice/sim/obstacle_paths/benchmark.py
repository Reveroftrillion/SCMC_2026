import sys,time,contextlib,io
sys.path.insert(0,'src/foscar_planning/test')
from test_tunnel_avoidance import context
from foscar_planning.states.avoid import Avoid
from foscar_planning.state import EventInfo
for count in [1,3,5]:
 a=Avoid(tunnel_extra_margin_m=0.);c=context();c.events=[EventInfo(type='OBSTACLE',stamp=1,s=40+i*6,d=(-1)**i*.5,size_x=1,size_y=1) for i in range(count)]
 ts=[]
 for tick in range(5):
  c.now+=.1
  for e in c.events:e.stamp=c.now;e.s+=.01
  t=time.perf_counter()
  with contextlib.redirect_stdout(io.StringIO()): a.check_entry(c);out=a.compute_target(c)
  ts.append(round((time.perf_counter()-t)*1000,1))
 print(count,ts,'blocked',a.slalom.blocked_at,'groups',len(a.slalom.groups),flush=True)
