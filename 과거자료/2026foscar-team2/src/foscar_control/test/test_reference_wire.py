from types import SimpleNamespace
from foscar_control.core import Target
from foscar_control.follower import Follower
from foscar_interfaces.msg import DrivingTarget

def node():return SimpleNamespace(target=None,target_arrival=0.,target_stamp=-float('inf'),fresh=lambda msg:True)
def message(t=1):
    m=DrivingTarget();m.header.stamp.sec=t;m.header.frame_id='map'
    m.v_max=5.;m.stop_s=float('nan');m.reference_id='curve'
    m.reference_global_s=[0.,1.,2.];m.reference_x=[0.,1.,2.];m.reference_y=[0.,0.,0.]
    return m

def test_atomic_reference_and_speed_roundtrip_rejects_older_packet():
    n=node();Follower.on_target(n,message(2))
    assert n.target.reference.id=='curve' and n.target.v_max==5
    old=message(1);old.v_max=30.
    Follower.on_target(n,old)
    assert n.target.v_max==5

def test_malformed_update_stops_on_previous_geometry():
    n=node();Follower.on_target(n,message())
    bad=message(2);bad.reference_x=[0.,2.,4.]
    Follower.on_target(n,bad)
    assert n.target.v_max==0 and n.target.reference.id=='curve'

def test_release_and_reference_cannot_be_sent_together():
    n=node();m=message();m.reference_release=True
    Follower.on_target(n,m)
    assert n.target.v_max==0 and n.target.reference is None

def test_clock_rewind_allows_new_epoch_but_not_old_packets():
    n=node();clock=[10.]
    n.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=int(clock[0]*1e9)))
    Follower.on_target(n,message(10))
    clock[0]=1.
    m=message(1);m.reference_id='new';m.v_max=3.
    Follower.on_target(n,m)
    assert n.target.reference.id=='new' and n.target_stamp==1.
    stale=message(0);Follower.on_target(n,stale)
    assert n.target.reference.id=='new'
