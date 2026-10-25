"""Jerk-limited braking of a speed request (user 2026-09-28/29).

Follow a gentle curve into the brake, but reach the braking finally wanted: when the request drops below our
speed, the shaped request starts from our speed and falls with a deceleration that builds up and eases off at
`jerk`, up to `max_decel`, until it meets the request. The caller decides when not to shape (a body inside
emergency range, a true stop): pass exempt=True and the request goes through unchanged.
"""
import math


class JerkBrake:
    def __init__(self, jerk=4., max_decel=8.):
        self.jerk, self.max_decel = jerk, max_decel
        self.state = None          # (shaped speed, deceleration (<= 0), stamp)

    def reset(self):
        self.state = None

    def apply(self, cap, speed, now, exempt=False):
        if exempt or not math.isfinite(cap) or cap >= speed-.3:
            self.state = None;return cap
        v, a, t0 = self.state if self.state is not None else (max(0., speed), 0., now)
        dt = min(max(0., now-t0), .2)
        want = max(-self.max_decel, min(0., cap-v))          # 1/s toward the request, at most max_decel
        a = max(want, a-self.jerk*dt) if want < a else min(want, a+self.jerk*dt)
        v = max(cap, v+a*dt);self.state = (v, a, now)
        return v
