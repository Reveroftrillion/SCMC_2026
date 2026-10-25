"""Independent geometry and front/rear axle contract checks for the batch probe."""
import math

import pytest

import probe_tunnel_matrix as probe
from ctrl_planning.ctrl_state import Target


def test_sat_distinguishes_gap_and_touching_rotated_body():
    box = [[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]]
    assert not probe.overlap(box, [[2., 0.], [3., -1.], [4., 0.], [3., 1.]])
    assert probe.overlap(box, [[1., 0.], [2., -1.], [3., 0.], [2., 1.]])


def test_quintic_chain_is_continuous_at_holds_and_returns():
    target = Target(chain=((2., 10., 10.), (0., 30., 10.)))
    assert probe.lateral(target, 5.) == (0., 0.)
    assert probe.lateral(target, 15.) == pytest.approx((1., .375))
    assert probe.lateral(target, 20.) == (2., 0.)
    assert probe.lateral(target, 25.) == (2., 0.)
    assert probe.lateral(target, 35.) == pytest.approx((1., -.375))
    assert probe.lateral(target, 40.) == (0., 0.)


def test_front_wheel_stop_is_converted_before_body_sweep():
    body = probe.base.Body(s=20., length=2., width=1.)
    frame = probe.base.Frame(1., (body,), s=10.)
    # front_s=17.5 -> rear_s=14.5 -> bumper_s=18.3 < body_s_min=19
    target = Target(v_max=5., stop_s=17.5)
    assert probe.sampled_overlap(target, frame, (body,), 1.892, 3.8, .835) is None
    # Ignoring the finite stopping point would cross the box.
    target.stop_s = math.nan
    assert probe.sampled_overlap(target, frame, (body,), 1.892, 3.8, .835)['body'] == body.key


def test_endpoint_is_checked_without_sampling_beyond_the_stop():
    body = probe.base.Body(s=20., length=2., width=1.)
    frame = probe.base.Frame(1., (body,), s=10.)
    # The exact endpoint at rear_s=15.2 puts the front bumper on the body.
    target = Target(v_max=5., stop_s=18.2)
    assert probe.sampled_overlap(target, frame, (body,), 1.892, 3.8, .835) is not None


def test_full_target_checker_retains_far_obstacle_overlaps():
    body = probe.base.Body(s=1950., length=2., width=1.)
    frame = probe.base.Frame(1., (body,), s=1880.)
    assert probe.sampled_overlap(Target(v_max=5.), frame, (body,), 1.892, 3.8, .835) is not None
