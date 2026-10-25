import math
from foscar_planning.boundary_motion import BoundaryMotion


def test_boundary_speed_does_not_follow_changing_visible_center():
    motion=BoundaryMotion()
    for t in (0.,.12,.24,.36):
        motion.observe(t,10.+4.*t)
    assert motion.ready and abs(motion.speed-4.)<1e-8
    motion.observe(.36,99.)  # same sweep is never repeated evidence
    assert abs(motion.speed-4.)<1e-8


def test_occlusion_jump_and_long_gap_reset_boundary_motion():
    motion=BoundaryMotion()
    for t in (0.,.12,.24,.36):motion.observe(t,10.+4.*t)
    motion.observe(.48,20.)
    assert not motion.ready
    motion.observe(1.1,22.)
    assert not motion.ready


def test_absence_of_confirmed_edge_cannot_keep_old_speed_ready():
    motion=BoundaryMotion()
    for t in (0.,.12,.24,.36):motion.observe(t,10.+4.*t)
    motion.observe(.5,None)
    assert not motion.ready


def test_approaching_edge_does_not_average_away_recent_fast_motion():
    # Map-arc endpoint samples at the erroneous GO in run_20260925_213913.
    motion=BoundaryMotion()
    for t,s in ((0.,-45.096072675),(.1232284,-44.979509649),
                (.2453506,-44.632569163),(.355772,-43.973271062)):
        motion.observe(t,s)
    assert motion.ready and 3.<motion.speed<3.2
    assert motion.approach_speed>=5.9


def test_constant_approach_has_no_extra_speed_and_reset_forgets_fast_sample():
    motion=BoundaryMotion()
    for t in (0.,.12,.24,.36):motion.observe(t,10.+2.*t)
    assert abs(motion.approach_speed-2.)<1e-8
    motion.observe(.5,None)
    for t in (.6,.72,.84,.96):motion.observe(t,20.+t)
    assert motion.ready and abs(motion.approach_speed-1.)<1e-8
