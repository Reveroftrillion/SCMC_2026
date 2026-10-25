from ctrl_planning.ctrl_following_policy import following_speed


def test_closer_moving_follow_target_does_not_force_braking_at_four_meter_gap():
    assert following_speed(5.,5.,4.)==5.375


def test_short_gap_reduces_speed_and_stopped_lead_never_requests_reverse():
    assert following_speed(5.,5.,1.)<5.
    assert following_speed(0.,5.,.1)==0.


def test_comfort_gap_at_equal_speed():
    assert following_speed(4.,4.,2.9)==4.
