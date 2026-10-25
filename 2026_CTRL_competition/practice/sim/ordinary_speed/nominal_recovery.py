"""Standalone nominal recovery experiment. Production controller is not modified.
Model: robust fit to run_20261006_144029; extra actuator lag 0.08 s.
"""
import json
import math

results=[]
for dt in [.02,.05]:
    for kp,horizon in [(.2,0),(.4,.35),(.6,.5)]:
        speed=52/3.6
        integral=.3
        pedal=(.001617000339*speed**2-.07634462)/3.46595791
        actual=pedal
        previous=speed
        filtered=0.
        history=[]
        distance=0.
        reached=None
        peak=0.
        for i in range(round(10/dt)):
            filtered+=dt/(.15+dt)*((speed-previous)/dt-filtered)
            previous=speed
            error=60/3.6-speed
            integral=max(-.3,min(.3,integral+error*dt))
            command=max(0.,min(1.,kp*(error-max(filtered,0.)*horizon)+.1*integral))
            pedal=min(command,pedal+3*dt)
            history.append(pedal)
            # Fractional delay interpolation, fixed prehistory at initial steady pedal.
            at=i-.125/dt
            low=max(0,math.floor(at));high=max(0,math.ceil(at))
            delayed=history[low]+(history[high]-history[low])*(at-math.floor(at))
            actual+=dt/(.08+dt)*(delayed-actual)
            speed=max(0.,speed+dt*(3.46595791*actual+.07634462-.001617000339*speed**2))
            distance+=speed*dt
            peak=max(peak,speed*3.6)
            if reached is None and speed>=58.5/3.6:reached=(i+1)*dt
        results.append(dict(dt=dt,kp=kp,horizon=horizon,t58_5=reached,peak=peak,mean10=distance/10*3.6))
print(json.dumps(results,indent=2))
