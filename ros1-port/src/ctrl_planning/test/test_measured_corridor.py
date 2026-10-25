from types import SimpleNamespace
import numpy as np
from ctrl_control.ctrl_core import Route
from ctrl_planning.ctrl_measured_corridor import corridor_distance, distance_speed


def context(t=1.,s=0.,v=4.):
    return SimpleNamespace(route=Route(np.c_[np.arange(100.),np.zeros(100)]),s=s,s_offset=0.,now=t,v=v)


def test_measured_rear_edge_distance_ignores_hidden_vehicle_length_and_off_path_body():
    ctx=context()
    assert abs(corridor_distance(ctx,[[10,-1],[20,-1],[20,1],[10,1]])-6.2)<.3
    assert corridor_distance(ctx,[[10,3],[20,3],[20,5],[10,5]]) is None


def test_wide_body_crossing_path_is_not_missed_when_corners_are_outside_corridor():
    assert corridor_distance(context(),[[6,-4],[8,-4],[8,4],[6,4]])<2.5


def test_zero_width_observed_surface_does_not_fill_entire_route():
    assert corridor_distance(context(),[[10,0],[20,0],[20,0],[10,0]])>5.


def test_distance_rate_follows_equal_speed_without_object_identity():
    history=[]
    for i in range(7):
        cap,rate=distance_speed(history,context(1.+i*.1,s=i*.4),4.,5.56)
    assert rate is not None and abs(rate-4.)<.01
    assert cap>=4.


def test_static_close_body_stops_and_missing_distance_discards_history():
    history=[]
    assert distance_speed(history,context(),.3,5.56)[0]==0.
    assert distance_speed(history,context(2.),None,5.56)[0]==5.56
    assert history==[]
