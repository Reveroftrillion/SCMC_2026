"""The simple tunnel planner: where its moves may start."""
from dataclasses import dataclass

from foscar_planning.tunnel_simple import plan


@dataclass
class Box:
    s_min: float
    s_max: float
    d_min: float
    d_max: float


def make_plan(groups, feasible, **kwargs):
    return plan(groups, feasible, s_now=0., d_now=0., v=4., clear_s=lambda b: b.s_max + 1.,
                front=2., rear=1., half=.5, min_curve=4., curve_time=2., min_move=2., **kwargs)


def test_no_move_starts_before_the_tunnel_when_our_line_is_clear_to_it():
    # lane changes are scored outside the tunnel (GPS stops there, user 2026-10-10)
    result = make_plan([[Box(40., 42., -.5, .5)]], lambda _: [2.], move_from=20.)
    assert result.blocked_at is None
    assert result.starts[0] >= 20.
    assert make_plan([[Box(40., 42., -.5, .5)]], lambda _: [2.]).starts[0] < 20.


def test_an_obstacle_before_the_tunnel_is_still_avoided():
    result = make_plan([[Box(15., 17., -.5, .5)]], lambda _: [2.], move_from=20.)
    assert result.blocked_at is None
    assert result.starts[0] < 15.


def test_a_clear_offset_is_held_instead_of_moving_toward_a_partly_seen_obstacle():
    # A barrier's first visible piece is right of the route; we are already passing on its left.
    # Moving 3->1 and then 1->0 travels 3 m; holding 3 and returning once travels the same distance
    # without a second curve and leaves room when more of the barrier is seen.
    result=plan([[Box(40.,42.,-1.5,-.5)]],lambda _: [1.,3.],s_now=20.,d_now=3.,v=4.,
                clear_s=lambda b:b.s_max+1.,front=2.,rear=1.,half=.5,min_curve=4.,
                curve_time=2.,min_move=2.,hold_tol=.125)
    assert result.blocked_at is None
    assert result.offsets[0]==3. and result.lengths[0]==0.


def test_a_held_offset_that_meets_a_new_obstacle_is_not_kept():
    result=plan([[Box(40.,42.,2.5,3.5)]],lambda _: [1.,3.],s_now=20.,d_now=3.,v=4.,
                clear_s=lambda b:b.s_max+1.,front=2.,rear=1.,half=.5,min_curve=4.,
                curve_time=2.,min_move=2.,hold_tol=.125)
    assert result.blocked_at is None
    assert result.offsets[0]==1.
