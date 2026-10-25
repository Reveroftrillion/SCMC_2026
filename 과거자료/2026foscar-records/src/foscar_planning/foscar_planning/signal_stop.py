"""Camera-only signal memory, scoped to the next calibrated stop pose.

No time expiry of accepted colors: this is the user's explicit loss policy.
Only fresh observations may change that memory. Target.stop_s uses front axle.
"""
import math
from dataclasses import replace

COLORS={'red','yellow','green','left','green_left','red_left'}

class SignalStopPolicy:
    def __init__(self, lines, front_axle=3.0, pass_margin=2.0):
        self.lines=sorted(lines,key=lambda x:x['stop_rear_s'])
        self.front_axle=front_axle;self.pass_margin=pass_margin
        self.reset()
        for line in self.lines:
            if not (math.isfinite(line['start_s']) and math.isfinite(line['stop_rear_s']) and line['start_s']<line['stop_rear_s']):
                raise ValueError('invalid stop calibration')
    def reset(self):
        self.key=None;self.color=None;self.last_s=None;self.last_stamp=-math.inf
    def current(self,s):
        if not math.isfinite(s):return None
        if self.last_s is not None and (s<self.last_s-10 or s>self.last_s+80):self.reset()
        self.last_s=s
        line=next((x for x in self.lines if s<=x['stop_rear_s']+self.pass_margin),None)
        key=line['id'] if line else None
        if key!=self.key:self.key=key;self.color=None;self.last_stamp=-math.inf
        return line if line and s>=line['start_s'] else None
    def go(self,line):
        return self.key==line['id'] and self.color in ({'green','green_left'} if line['maneuver']=='straight'
                                                       else {'left','green_left','red_left'})
    def lateral_hold_s(self,s):
        """Rear-axle stop pose of the next line not yet reached, unless its go colour is confirmed (NaN).

        Holding keeps the car straight so it stops square at the line. Once the car will not stop,
        holding only shortens the swerve: at 58 km/h on green the box was seen 10 m before the line,
        the curve after it was too short and the car hit the box (2026-09-28 forced-signal run A2).
        """
        if not math.isfinite(s):return math.nan
        line=next((x for x in self.lines if s<x['stop_rear_s']),None)
        return math.nan if line is None or self.go(line) else line['stop_rear_s']
    def observe(self,key,color,s,stamp,now=None):
        line=self.current(s);now=stamp if now is None else now
        if not line or key!=line['id'] or color not in COLORS:return False
        if not math.isfinite(stamp) or not -.05<=now-stamp<=1.0 or stamp<=self.last_stamp:return False
        self.color=color;self.last_stamp=stamp;return True
    def apply(self,target,s,now):
        line=self.current(s)
        if not line or self.color is None:return target
        if self.go(line):return target
        stop=line['stop_rear_s']+self.front_axle
        if math.isfinite(target.stop_s):stop=min(stop,target.stop_s)
        return replace(target,stop_s=stop,
                       v_max=min(target.v_max,0.) if s>=line['stop_rear_s']-.3 else target.v_max,
                       state=target.state+'|SIGNAL_STOP')
