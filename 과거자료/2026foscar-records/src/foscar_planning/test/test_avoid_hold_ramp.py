import math
from foscar_planning.states.avoid import Avoid
from foscar_planning.state import Context


def test_swerve_hold_rises_slowly_faster_near_the_tunnel_end():
    # 2026-09-28: frozen at the measured speed each tick, the hold ratcheted 41 -> 29 km/h through
    # the tunnel slalom, then the car launched at pedal 0.8 on exit.
    def ramp(s):
        a=Avoid();v=11.
        a.held_speed(Context(s=s,x=0,y=0,yaw=0,v=v,now=0.))
        return a.held_speed(Context(s=s,x=0,y=0,yaw=0,v=v,now=.5))-v
    early,late=ramp(1880.),ramp(1995.)
    assert 0<early<late
    assert abs(early-.3*.5)<.05 and late<=1.5*.5+1e-9


def test_swerve_hold_never_runs_more_than_a_second_ahead_of_the_car():
    a=Avoid();a.held_speed(Context(s=1990.,x=0,y=0,yaw=0,v=11.,now=0.))
    held=a.held_speed(Context(s=1990.,x=0,y=0,yaw=0,v=8.,now=10.))   # braked by something else
    assert held<=8.+1.5*1.+1e-9
