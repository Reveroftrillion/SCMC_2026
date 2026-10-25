from types import SimpleNamespace as NS
from ctrl_camera_train.ctrl_survey import ready_to_save

def sample(stamp=1000000):
    return NS(header=NS(stamp=NS(sec=1, nanosec=stamp)))

def test_save_requires_fresh_stationary_synchronized_data():
    samples = {k:(10., sample()) for k in ('odom','s','status','rgb','sem')}
    assert ready_to_save(samples, 10.1, 9.)
    assert not ready_to_save(samples, 10.1, None)
    assert not ready_to_save(samples, 10.1, 10.)
    assert not ready_to_save(samples, 11., 9.)
    samples['sem'] = (10., sample(20000000))
    assert not ready_to_save(samples, 10.1, 9.)
    del samples['status']
    assert not ready_to_save(samples, 10.1, 9.)
