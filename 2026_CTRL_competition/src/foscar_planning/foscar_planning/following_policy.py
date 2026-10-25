"""Shared circular lead-following target for admission preview and live control.

This comfort gap does not replace the independent body collision checks.
"""


def following_speed(lead_speed, ego_speed, gap, tuck=0.):
    """tuck 0..1 (user 2026-09-29): entering the ring we may tuck in close behind the lead (1.2 m + 0.1 s x speed,
    above the 0.8 m too-close stop), then reopen to the ordinary gap as tuck falls back to 0."""
    v=max(0.,ego_speed)
    desired=(1.5+.35*v)*(1.-tuck)+(1.2+.1*v)*tuck
    return max(0.,lead_speed+.5*(gap-desired))
