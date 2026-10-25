"""Motion of a supported endpoint in unwrapped map arc coordinates."""
from collections import deque
import numpy as np


class BoundaryMotion:
    def __init__(self):
        self.samples=deque(maxlen=10)
        self.stamp=-float('inf');self.ready=False;self.speed=0.;self.residual=0.

    @property
    def approach_speed(self):
        """Include observed fast approach, even while the fit is catching up.

        A partially revealed face can move slowly then expose its true leading
        edge. The least-squares mean alone understates rear arrival in that
        case. Retain the fastest measured secant in the same short, continuous
        observation window. This is not a speed multiplier or an old-track
        prediction; absence/jumps reset the underlying samples as before.
        """
        rows=list(self.samples)
        rates=[(s1-s0)/(t1-t0) for i,(t0,s0) in enumerate(rows)
               for t1,s1 in rows[i+1:] if t1-t0>=.05]
        return max(self.speed,max(rates,default=0.))

    def observe(self,stamp,position):
        if stamp<=self.stamp:return
        self.stamp=stamp;self.ready=False
        if position is None or not np.isfinite(position):
            self.samples.clear();return
        if self.samples:
            dt=stamp-self.samples[-1][0]
            rate=(position-self.samples[-1][1])/dt
            if dt>.3 or not -.5<=rate<=15.:self.samples.clear()
        self.samples.append((stamp,position))
        while self.samples and stamp-self.samples[0][0]>.6:self.samples.popleft()
        if len(self.samples)<3 or stamp-self.samples[0][0]<.2:return
        rows=np.asarray(self.samples);t=rows[:,0]-stamp
        speed,offset=np.polyfit(t,rows[:,1],1)
        residual=float(np.max(abs(rows[:,1]-(speed*t+offset))))
        self.residual=residual
        self.speed=max(0.,float(speed))
        self.ready=bool(-.2<=speed<=15. and residual<=.25)
