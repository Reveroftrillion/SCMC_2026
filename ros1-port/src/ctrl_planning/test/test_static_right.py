"""The approved fixed-course change goes right without weakening map/body gates."""
import numpy as np

from ctrl_control.ctrl_core import Controller
from ctrl_planning.ctrl_mapped_avoidance import paved_moves
from ctrl_planning.ctrl_static_course import MOVES
from test_static_course import course_context
from ctrl_planning.states.ctrl_avoid import Avoid


def test_static_course_uses_right_lane_and_returns_before_cp4():
    ctx = course_context()
    avoid = Avoid(static_course=True)
    target = avoid.course.target(ctx, avoid)
    assert target.lat_offset < 0.
    assert target.chain == MOVES
    assert target.return_s + target.return_length_m < 345.14052925086577
    assert target.v_max > 0.


def test_sign_flip_of_old_left_chain_is_rejected_by_unchanged_body_map_check():
    ctx = course_context()
    avoid = Avoid(static_course=True)
    assert not paved_moves(ctx, 0., ((-4.,310.,12.),(0.,333.,18.)),290.,370.,
                           avoid.front_m,avoid.rear_overhang_m,avoid.static_planner.half)
    assert paved_moves(ctx, 0., MOVES,290.,370.,
                       avoid.front_m,avoid.rear_overhang_m,avoid.static_planner.half)


def test_right_curve_is_continuous_and_within_existing_steering_curvature():
    avoid = Avoid(static_course=True)
    previous = 0.
    for offset, start, length in MOVES:
        coefficients = Controller.curve_coefficients(previous, offset, 0., 0., length)
        before = Controller.curve_state(coefficients, 0., length)
        after = Controller.curve_state(coefficients, 1., length)
        np.testing.assert_allclose(before, [previous,0.,0.], atol=1e-10)
        np.testing.assert_allclose(after, [offset,0.,0.], atol=1e-10)
        assert avoid.swerve_curvature(length, abs(offset-previous)) < 1./4.3
        previous = offset
