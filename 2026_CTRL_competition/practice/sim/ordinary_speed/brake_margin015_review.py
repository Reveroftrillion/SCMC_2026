from pathlib import Path
import types,sys,itertools,json,numpy as np
source=Path('practice/sim/ordinary_speed/brake_reference_core.txt').read_text()
rows=[]
for label,condition in [('current','required_decel > stop_decel'),('margin_015','required_decel > stop_decel')]:
 mod=types.ModuleType('foscar_control.offline_brake');sys.modules[mod.__name__]=mod
 exec(compile(source.replace('required_decel > stop_decel',condition).replace('remaining-.25-max(0., speed)*.15','remaining-.15-max(0., speed)*.15') if label=='margin_015' else source,'<offline-brake>','exec'),mod.__dict__)
 for entry,x0,v0,u0 in [('baseline',175.241724,14.498667,.4779),('A',176.046623,15.265929,.3953),('failed',177.146233,15.343433,.37)]:
  for gain,delay,lag,noise,nonlinear in itertools.product([13.,15.,16.],[.075,.125,.15,.2],[0.,.05],[0.,.08],[False,True]):
   c=mod.Controller(mod.Route(np.c_[np.arange(1001.),np.zeros(1001)]),speed_kmh=60,continuous=True,accel_max=1)
   v=v0;x=x0;stop=202.485;dt=.05;ts=[-1.,0.];us=[u0,u0];bs=[0.,0.];au=u0;ab=0.;rng=np.random.default_rng(1);done=False
   for k in range(300):
    t=k*dt;o=c.step(x,0,0,max(0,v+rng.normal(0,noise)),2,4,True,dt,mod.Target(state='CRUISE|SIGNAL_STOP',v_max=60/3.6,stop_s=stop+mod.FRONT_AXLE))
    ts.append(t+dt);us.append(o.accel);bs.append(o.brake);au+=dt/(.08+dt)*(np.interp(t+dt-.125,ts,us)-au)
    delayed=np.interp(t+dt-delay,ts,bs);ab=delayed if lag==0 else ab+dt/(lag+dt)*(delayed-ab)
    decel=np.interp(ab,[0,.3,.5,1],[0,gain*.3,5,9]) if nonlinear else gain*ab
    v=max(0,v+dt*(3.466*au+.0763-.001617*v*v-decel));x+=v*dt
    if v<.03:done=True;break
   rows.append(dict(label=label,entry=entry,gain=gain,delay=delay,lag=lag,noise=noise,nonlinear=nonlinear,error=x-stop,time=(k+1)*dt,done=done))
for label in ['current','margin_015']:
 for entry in ['baseline','A','failed']:
  a=[r for r in rows if r['label']==label and r['entry']==entry];nom=[r for r in a if r['gain']==15 and r['delay']==.125 and r['lag']==0 and r['noise']==0 and not r['nonlinear']][0]
  print(label,entry,'error range',min(r['error'] for r in a),max(r['error'] for r in a),'time mean',np.mean([r['time'] for r in a]),'nominal',nom)
json.dump(rows,open('practice/sim/ordinary_speed/brake_margin015_review.json','w'),indent=2)
