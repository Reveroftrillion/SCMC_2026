import numpy as np,itertools,json
from foscar_control.core import Controller,Route,Target,FRONT_AXLE
rows=[]
for gain,delay,lag,noise,nonlinear in itertools.product([13.,15.,16.],[.075,.125,.15,.2],[0.,.05],[0.,.08],[False,True]):
 c=Controller(Route(np.c_[np.arange(1001.),np.zeros(1001)]),speed_kmh=60,continuous=True,accel_max=1)
 v=15.343432956;x=177.146232746;stop=202.485;dt=.05;ts=[-1.,0.];us=[.37,.37];bs=[0.,0.];au=.37;ab=0.;rng=np.random.default_rng(1);peak_b=0.;stopped=False
 for k in range(300):
  t=k*dt;o=c.step(x,0,0,max(0,v+rng.normal(0,noise)),2,4,True,dt,Target(state='CRUISE|SIGNAL_STOP',v_max=60/3.6,stop_s=stop+FRONT_AXLE))
  ts.append(t+dt);us.append(o.accel);bs.append(o.brake);peak_b=max(peak_b,o.brake)
  au+=dt/(.08+dt)*(np.interp(t+dt-.125,ts,us)-au)
  delayed_b=np.interp(t+dt-delay,ts,bs);ab=delayed_b if lag==0 else ab+dt/(lag+dt)*(delayed_b-ab)
  decel=gain*ab
  if nonlinear:decel=np.interp(ab,[0,.3,.5,1],[0,gain*.3,5,9])
  v=max(0,v+dt*(3.466*au+.0763-.001617*v*v-decel));x+=v*dt
  if v<.03:stopped=True;break
 rows.append(dict(gain=gain,delay=delay,lag=lag,noise=noise,nonlinear=nonlinear,error=x-stop,peak_brake=peak_b,stopped=stopped))
for nonlinear in [False,True]:
 subset=[r for r in rows if r['nonlinear']==nonlinear];print('nonlinear',nonlinear,'min',min(subset,key=lambda r:r['error']),'max',max(subset,key=lambda r:r['error']),'positive',sum(r['error']>0 for r in subset))
json.dump(rows,open('/home/foscar/Desktop/2026Morai/2026foscar-competition/practice/sim/ordinary_speed/brake_sweep.json','w'),indent=2)
