from ctrl_planning.ctrl_state import EventInfo, event_key


def test_moving_track_replaces_old_position_even_when_kind_changes():
    old = EventInfo(type='OBSTACLE', stamp=1., s=460., d=-1., value='lidar:session:3')
    new = EventInfo(type='PEDESTRIAN', stamp=1.1, s=460., d=1., value=old.value)
    cache = {event_key(old): old}
    cache[event_key(new)] = new
    assert list(cache.values()) == [new]
    new.value = 'lidar:other_session:3'
    assert event_key(old) != event_key(new)


def test_legacy_and_traffic_light_events_keep_spatial_keys():
    a = EventInfo(type='TRAFFIC_LIGHT', stamp=1., s=200., d=0., value='RED')
    b = EventInfo(type='TRAFFIC_LIGHT', stamp=2., s=200., d=0., value='GREEN')
    assert event_key(a) == event_key(b)


def test_offpath_observation_replaces_driving_event_and_rejects_late_packet():
    import rclpy
    from ctrl_interfaces.msg import Event
    from ctrl_planning.ctrl_behavior_node import BehaviorNode
    rclpy.init()
    node=BehaviorNode()
    try:
        e=Event();e.type='VEHICLE';e.value='lidar:test:99';e.header.frame_id='map'
        e.header.stamp.sec=10
        node.on_event(e)
        assert next(iter(node.events.values())).path_relevant
        e.header.stamp.sec=11
        node.on_observation(e)
        assert not next(iter(node.events.values())).path_relevant
        e.header.stamp.sec=10
        node.on_event(e)
        assert not next(iter(node.events.values())).path_relevant
        assert next(iter(node.events.values())).stamp==11.
    finally:
        node.destroy_node();rclpy.shutdown()
