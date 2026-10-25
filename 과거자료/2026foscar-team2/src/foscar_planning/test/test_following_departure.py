import math
from types import SimpleNamespace
import numpy as np
from foscar_planning.circle_reference import CircularReference
from foscar_planning.circular_gap import RingTrack
from foscar_planning.following_departure import FollowingDeparture
from foscar_planning.state import Context,EventInfo
from foscar_control.core import Route


def test_departure_tracks_slow_front_instead_of_accelerating_through_it():
    # Straight route tangent to a sufficiently large circle approximates a lane.
    circle=CircularReference(np.array([0.,1000.]),1000.,0.,0.,1)
    route=Route(np.c_[np.arange(100.),np.zeros(100)])
    tr=RingTrack();e=EventInfo('VEHICLE',1.,x=14.,y=0.,vx=2.,size_x=4.8,size_y=2.,box_yaw=0.,value='lead')
    tr.update(e,circle);tr.ready=True;tr.speed=2.
    ctx=Context(0.,0.,0.,0.,0.,1.,route=route)
    model=FollowingDeparture(SimpleNamespace(max_speed=5.5),circle,[tr])
    empty=FollowingDeparture(SimpleNamespace(max_speed=5.5),circle,[])
    assert model.ego_pose(ctx,10.,1.5)[0]<empty.ego_pose(ctx,10.,1.5)[0]-5.
    assert model.cache[(1.,1.5)][-1][2]<3.


def test_measured_boosted_departure_fits_prediction_position_envelope():
    import json
    from pathlib import Path
    samples=json.loads((Path(__file__).parent/'data/entry_acceleration_20260925.json').read_text())['samples']
    route=Route(np.c_[np.arange(100.),np.zeros(100)])
    circle=CircularReference(np.array([0.,1000.]),1000.,0.,0.,1)
    ctx=Context(0.,0.,0.,0.,0.,0.,route=route)
    model=FollowingDeparture(SimpleNamespace(max_speed=2.78),circle,[])
    for sample in samples:
        positions=[]
        for accel in (1.5,1.8,2.2):
            model.ego_pose(ctx,sample['t'],accel)
            positions.append(model.cache[(0.,accel)][-1][1])
        assert min(positions)-.25<=sample['s']<=max(positions)+.25


def test_measured_braking_does_not_travel_beyond_conservative_prediction():
    import json
    from pathlib import Path
    from foscar_planning.following_departure import conservative_braking
    rows=json.loads((Path(__file__).parent/'data/entry_acceleration_20260925.json').read_text())['braking_samples']
    speed=rows[0]['v'];distance=0.;time=0.
    for row in rows:
        while time<row['t']-1e-8:
            dt=min(.01,row['t']-time)
            new_speed=speed if time<.15 else max(0.,speed-conservative_braking(speed,0.)*dt)
            distance+=(speed+new_speed)*dt/2;speed=new_speed;time+=dt
        assert row['s']<=distance+.25


def test_braking_hypotheses_bound_both_front_and_rear_measured_progress():
    import json
    from pathlib import Path
    from foscar_planning.following_departure import braking_response
    rows=json.loads((Path(__file__).parent/'data/entry_acceleration_20260925.json').read_text())['braking_samples']
    distances=[]
    for accel in (1.5,2.2):
        speed=rows[0]['v'];distance=0.;time=0.;profile=[]
        for row in rows:
            while time<row['t']-1e-8:
                dt=min(.01,row['t']-time)
                decel,reaction=braking_response(speed,0.,accel)
                new_speed=speed if time<reaction else max(0.,speed-decel*dt)
                distance+=(speed+new_speed)*dt/2;speed=new_speed;time+=dt
            profile.append(distance)
        distances.append(profile)
    for i,row in enumerate(rows):
        assert distances[0][i]-.25<=row['s']<=distances[1][i]+.25


def test_departure_following_uses_observed_rear_face_not_centered_hidden_body():
    circle=CircularReference(np.array([0.,1000.]),1000.,0.,0.,1)
    route=Route(np.c_[np.arange(100.),np.zeros(100)])
    ctx=Context(0.,0.,0.,0.,0.,1.,route=route)
    def rollout(bounds):
        tr=RingTrack()
        tr.update(EventInfo('VEHICLE',1.,x=14.,y=0.,vx=2.,size_x=.3,size_y=2.,box_yaw=0.,value='lead'),circle)
        tr.ready=True;tr.speed=2.;tr.has_polygon=True
        tr.arc_lo=-.15;tr.arc_hi=.15;tr.observed_length=.3;tr.body_bounds=bounds
        model=FollowingDeparture(SimpleNamespace(max_speed=5.5),circle,[tr])
        return model.ego_pose(ctx,8.,1.8)[0]
    # The same observed face can anchor either end. Hidden length must be on
    # the known side, matching the geometry used by actual following control.
    assert rollout((-.2,4.8))>rollout((-4.8,.2))+2.


def test_departure_respects_front_gap_before_body_touches_ring():
    # Actual local following caps speed immediately after permission, even
    # while ego remains outside the ring. The preview must do the same.
    circle=CircularReference(np.array([0.,1000.]),1000.,0.,0.,1)
    route=Route(np.c_[np.arange(100.),np.full(100,-5.)])
    tr=RingTrack()
    tr.update(EventInfo('VEHICLE',1.,x=6.,y=0.,vx=2.,size_x=4.8,size_y=2.,box_yaw=0.,value='lead'),circle)
    tr.ready=True;tr.speed=2.
    ctx=Context(0.,0.,-5.,0.,0.,1.,route=route)
    model=FollowingDeparture(SimpleNamespace(max_speed=5.5),circle,[tr],follow_before_contact=True)
    model.ego_pose(ctx,1.,2.2)
    # No intersection yet, but a nearby departing front already constrains
    # speed. The old radial trigger left every wanted sample at max_speed.
    assert model.cache[(1.,2.2)][5][3]<3.


def test_local_admission_rollout_does_not_brake_for_rear_overtaking_in_prediction():
    circle=CircularReference(np.array([0.,1000.]),1000.,0.,0.,1)
    route=Route(np.c_[np.arange(100.),np.zeros(100)])
    tr=RingTrack()
    tr.update(EventInfo('VEHICLE',1.,x=-3.,y=0.,vx=7.,size_x=4.8,size_y=2.,value='rear'),circle)
    tr.ready=True;tr.speed=7.
    ctx=Context(0.,0.,0.,0.,2.,1.,route=route)
    model=FollowingDeparture(SimpleNamespace(max_speed=5.5),circle,[tr],follow_before_contact=True)
    empty=FollowingDeparture(SimpleNamespace(max_speed=5.5),circle,[],follow_before_contact=True)
    # Rear collision is checked by CircularGap independently. It must not also
    # create hypothetical front braking and a self-fulfilling rear conflict.
    assert np.allclose(model.ego_pose(ctx,5.,1.5),empty.ego_pose(ctx,5.,1.5))


def test_point_six_launch_measured_progress_fits_updated_envelope():
    import json
    from pathlib import Path
    from foscar_planning.circular_gap import CircularGap
    samples=json.loads((Path(__file__).parent/'data/entry_acceleration_060_20260925.json').read_text())['samples']
    route=Route(np.c_[np.arange(100.),np.zeros(100)])
    circle=CircularReference(np.array([0.,1000.]),1000.,0.,0.,1)
    ctx=Context(0.,0.,0.,0.,0.,0.,route=route)
    model=FollowingDeparture(SimpleNamespace(max_speed=5.5556),circle,[])
    for sample in samples:
        positions=[]
        for accel in CircularGap(circle).departure_accels:
            model.ego_pose(ctx,sample['t'],accel)
            positions.append(model.cache[(0.,accel)][-1][1])
        assert min(positions)-.25<=sample['s']<=max(positions)+.25


def test_calibrated_launch_lower_bound_no_longer_loses_half_first_second():
    from foscar_planning.circular_gap import CircularGap
    route=Route(np.c_[np.arange(100.),np.zeros(100)])
    circle=CircularReference(np.array([0.,1000.]),1000.,0.,0.,1)
    ctx=Context(0.,0.,0.,0.,0.,0.,route=route)
    model=FollowingDeparture(SimpleNamespace(max_speed=5.5556),circle,[])
    lower=min(CircularGap(circle).departure_accels)
    model.ego_pose(ctx,1.,lower)
    travel=model.cache[(0.,lower)][-1][1]
    # Recorded .60-throttle launch traveled1.330m at1s. Keep a lower bound
    # without the old0.25s delay/2.5m/s² assumption (~0.703m).
    assert 1.10<=travel<=1.33


def test_departure_preview_obeys_same_curvature_profile_as_controller():
    from foscar_control.core import Controller
    a=np.linspace(0.,2.,401)
    route=Route(np.c_[5.*np.cos(a),5.*np.sin(a)])
    circle=CircularReference(np.zeros(2),5.,0.,0.,1)
    ctx=Context(0.,5.,0.,math.pi/2,0.,0.,route=route)
    controller=Controller(route,speed_kmh=20.,continuous=True)
    model=FollowingDeparture(SimpleNamespace(max_speed=20./3.6),circle,[])
    model.ego_pose(ctx,3.,2.9)
    assert all(row[3]<=route.speed_at(row[1],controller.profile)+.02
               for row in model.cache[(0.,2.9)][1:])
    assert model.cache[(0.,2.9)][-1][2]<4.


def test_departure_profile_uses_controller_limits_and_invalidates_cached_settings():
    from foscar_control.core import Controller
    a=np.linspace(0.,2.,401);route=Route(np.c_[5.*np.cos(a),5.*np.sin(a)])
    circle=CircularReference(np.zeros(2),5.,0.,0.,1)
    predictor=SimpleNamespace(max_speed=20./3.6,departure_profile_limits=(1.5,1.2,.8),departure_cruise_speed=30./3.6)
    model=FollowingDeparture(predictor,circle,[])
    controller=Controller(route,speed_kmh=30.,a_lat=1.5,a_dec=1.2,a_acc=.8)
    assert abs(model.route_speed(route,4.)-min(predictor.max_speed,route.speed_at(4.,controller.profile)))<1e-9
    old=model.route_speed(route,4.)
    predictor.departure_profile_limits=(2.5,1.2,.8)
    assert model.route_speed(route,4.)>old
