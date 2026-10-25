from foscar_planning.rear_response import predict_rear_response


def test_following_response_is_delayed_and_deceleration_is_bounded():
    trace=[(i*.2,i*.4) for i in range(31)]
    out=predict_rear_response(-10.,5.,3.,trace,reaction_s=.4,decel=2.5)
    assert abs(out[.4]['s']-(-8.))<1e-6
    assert out[2.]['speed']<5.
    previous=5.
    for row in out.values():
        assert row['speed']>=0.
        assert previous-row['speed']<=.500001
        previous=row['speed']


def test_response_cannot_make_already_overlapping_entry_safe():
    trace=[(0.,0.),(.2,.2),(.4,.4)]
    out=predict_rear_response(-1.,5.,3.,trace)
    assert out[0.]['s']+3.>trace[0][1]


def test_handoff_allows_speed_difference_when_gap_absorbs_delayed_response():
    from foscar_planning.rear_response import handoff_reserve
    trace=[(i*.2,2.5*i*.2) for i in range(11)]
    assert handoff_reserve(-8.,3.,3.,trace)>0.


def test_handoff_rejects_closing_before_rear_can_respond():
    from foscar_planning.rear_response import handoff_reserve
    trace=[(i*.2,2.5*i*.2) for i in range(11)]
    assert handoff_reserve(-3.1,5.,3.,trace)<0.


def test_doubled_rear_speed_consumes_gap_without_changing_thresholds():
    from foscar_planning.rear_response import handoff_reserve
    trace=[(i*.2,2.5*i*.2) for i in range(11)]
    assert handoff_reserve(-6.,2.78,3.,trace)>0.
    assert handoff_reserve(-6.,5.56,3.,trace)<0.


def test_noncooperative_response_cannot_assume_rear_deceleration():
    from foscar_planning.rear_response import handoff_reserve
    trace=[(i*.2,2.5*i*.2) for i in range(11)]
    assert handoff_reserve(-6.,4.,3.,trace,decel=0.)<0.


def test_reaction_delay_uses_ego_and_rear_positions_from_same_time():
    trace=[(0.,0.),(.2,0.),(.4,0.),(.6,0.)]
    response=predict_rear_response(-10.5,3.,3.,trace)
    # At t=.4 the controller sees the t=0 gap 7.5m, not the t=.4 rear
    # position paired with the old ego position (which invents a smaller gap).
    assert abs(response[.6]['speed']-2.76)<1e-6


def test_handoff_cannot_authorize_a_truncated_preview():
    from foscar_planning.rear_response import handoff_reserve
    assert handoff_reserve(-100.,3.,3.,[(0.,0.),(.2,1.)])==-float('inf')
