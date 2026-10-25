"""Exercise the real follower tick with cached ROS messages, without publishing."""
from types import SimpleNamespace

import numpy as np
import pytest
from nav_msgs.msg import Odometry
from foscar_interfaces.msg import VehicleStatus
from foscar_control.core import Controller, Route, Target
from foscar_control.follower import Follower


def fixture(monkeypatch, state='CRUISE'):
    now = [10.]
    monkeypatch.setattr('foscar_control.follower.time.monotonic', lambda: now[0])
    route = Route(np.c_[np.arange(1001.), np.zeros(1001)])
    c = Controller(route, speed_kmh=60., continuous=True, accel_max=1.)
    odom = Odometry()
    odom.pose.pose.position.x = 100.
    odom.pose.pose.orientation.w = 1.
    odom.twist.twist.linear.x = 54./3.6
    status = VehicleStatus()
    status.ctrl_mode, status.gear = 2, 4
    out = []
    node = SimpleNamespace(odom=odom, status=status, controller=c, route=route,
        p=dict(sensor_timeout=1.5, sensor_mount_confirmed=True, target_timeout=.5,
               require_target=True, dr_speed_kmh=0.),
        arrivals=dict(odom=10., status=10.), previous_feedback=None,
        previous_tick=9.95, target=Target(state=state, v_max=60./3.6), target_arrival=10.,
        loc_mode='GPS', handshake_start=None,
        get_logger=lambda: SimpleNamespace(warn=lambda *a, **kw: None),
        publish_output=lambda output, *args: out.append(output))
    Follower.tick(node)
    assert out[-1].accel > 0.
    return now, node, out


@pytest.mark.parametrize('missing', ['status', 'odom'])
def test_cached_feedback_coasts_even_before_the_sensor_timeout(monkeypatch, missing):
    now, n, out = fixture(monkeypatch)
    now[0] += .05
    n.arrivals['odom' if missing == 'status' else 'status'] = now[0]
    Follower.tick(n)
    assert out[-1].state == 'CRUISE'
    assert out[-1].accel == 0.
    assert out[-1].brake == 0.
    # Same speed with genuinely new messages is valid feedback, not a stalled sensor.
    now[0] += .05
    n.arrivals = dict(odom=now[0], status=now[0])
    Follower.tick(n)
    assert 0. < out[-1].accel <= .15+1e-9


def test_control_loop_stall_does_not_reissue_ordinary_acceleration(monkeypatch):
    now, n, out = fixture(monkeypatch)
    now[0] += .3
    Follower.tick(n)
    assert out[-1].accel == 0.
    assert n.controller.previous_accel == 0.


def test_special_mission_retains_its_existing_cached_feedback_response(monkeypatch):
    now, n, out = fixture(monkeypatch, state='AVOID')
    now[0] += .05
    Follower.tick(n)
    assert out[-1].accel > 0.
