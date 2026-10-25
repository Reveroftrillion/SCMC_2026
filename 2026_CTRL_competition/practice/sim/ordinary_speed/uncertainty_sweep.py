"""Offline formula prototype. Step/recovery caps are test stimuli, never controller policy.
This approximates longitudinal feedback, not the complete route or controller.
"""
import itertools,json,math
import numpy as np
rows=[]
for kp,horizon in [(.2,0),(.4,.35),(.6,.5)]:
 for initial in [0,52/3.6]:
  for gain,drag,delay,tau,noise in itertools.product([2.8,3.52,4.3],[.2,.39,.6],[.05,.15,.3],[.05,.2],[0,.08]):
   dt=.05;speed=initial;integ=.3;pedal=drag*(speed/(60/3.6))**2/gain;actual=pedal;history=[];vel=[];prev=speed;filtered=0.;rng=np.random.default_rng(1);reach=None
   for i in range(800):
    tm=i*dt;target=60/3.6 if tm<20 else 30/3.6 if tm<30 else 0
    measured=speed+rng.normal(0,noise);filtered+=dt/(.15+dt)*((measured-prev)/dt-filtered);prev=measured
    error=target-measured;integ=max(-.3,min(.3,integ+error*dt))
    command=max(0,min(1,kp*(error-max(0,filtered)*horizon)+.1*integ)) if error>=-.4 and target>0 else 0
    pedal=min(command,pedal+3*dt);brake=min(1,max(0,-error*.08)) if error<-.4 else 0
    if target==0:pedal=0;brake=max(.15,brake);integ=0
    history.append((pedal,brake));at=i-delay/dt;lo=max(0,math.floor(at));hi=max(0,math.ceil(at));w=at-math.floor(at)
    delayed=[history[lo][j]+w*(history[hi][j]-history[lo][j]) for j in [0,1]]
    actual+=dt/(tau+dt)*(delayed[0]-actual)
    speed=max(0,speed+dt*(gain*actual-drag*(speed/(60/3.6))**2-12*delayed[1]));vel.append(speed)
    if reach is None and speed>=58.5/3.6:reach=(i+1)*dt
   rows.append(dict(kp=kp,horizon=horizon,initial=initial*3.6,gain=gain,drag=drag,delay=delay,tau=tau,noise=noise,peak=max(vel[:400])*3.6,t58_5=reach,at40=vel[-1]*3.6))
json.dump(rows,open('practice/sim/ordinary_speed/uncertainty_20hz.json','w'),indent=2)
for kp,horizon in [(.2,0),(.4,.35),(.6,.5)]:
 for initial in [0,52.]:
  subset=[r for r in rows if r['kp']==kp and abs(r['initial']-initial)<.1]
  print(kp,horizon,initial,'worst',max(subset,key=lambda r:r['peak']))
