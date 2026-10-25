"""Offline-testable decisions for the external simulator audit observer."""
import math
import struct


def decode_collision(raw):
    # Wire layout from supplied practice/sim/record_collision.py (read-only).
    if (len(raw) != 181 or raw[:15] != b'#CollisionData$'
            or raw[-2:] != b'\r\n' or struct.unpack_from('<i', raw, 15)[0] != 148):
        raise ValueError('invalid CollisionData framing')
    return [dict(object_type=v[0], object_id=v[1], position=list(v[2:5]),
                 global_offset=list(v[5:8]))
            for v in (struct.unpack_from('<hh6f', raw, 39+i*28) for i in range(5))]


def collision_slots(slots):
    # Supplied practice/avoid_test/avoid_mon.py: type -1 means no collision;
    # id 0 is an empty/non-object slot. Preserve raw slots in the audit log.
    return [v for v in slots if v['object_type'] >= 0 and v['object_id'] != 0]


def manual_park_evidence(samples, after, now):
    recent = [v for v in samples if v['time'] >= max(after, now-.7)]
    if (len(recent) >= 5 and now-recent[-1]['time'] < .3
            and recent[-1]['time']-recent[0]['time'] >= .4
            and len({v['stamp_hex'] for v in recent}) >= 2
            and all(math.isfinite(v['speed']) and abs(v['speed']) < .01
                    and v['mode'] == 1 and v['gear'] == 1 for v in recent)):
        return dict(samples=len(recent), duration_s=recent[-1]['time']-recent[0]['time'],
                    maximum_abs_speed_mps=max(abs(v['speed']) for v in recent),
                    last=recent[-1])
    return None


def input_loss_reason(now, started, wire_received, ros_received):
    """Abort loss of observation; missing GPS alone is valid inside the tunnel."""
    if now-started <= 20.:
        return None
    for port, reason in (('909', 'raw_vehicle_status_lost'),
                         ('9092', 'collision_monitor_lost')):
        if now-wire_received.get(port, started) > 1.:
            return reason
    for topic in ('/sensors/imu', '/sensors/lidar/points', '/perception/footprints',
                  '/sensors/camera/traffic/compressed', '/localization/odometry'):
        if now-ros_received.get(topic, started) > 3.:
            return 'required_sensor_or_localization_lost: '+topic
    return None


def missing_required_nodes(now, started, nodes):
    if now-started <= 20.:
        return []
    expected = {'/sensor_bridge', '/localization', '/path_follower', '/behavior_node',
                '/lidar_bridge', '/obstacle_node', '/camera_bridge', '/yolo_signal_node'}
    return sorted(expected-set(nodes))


class DepartureGuard:
    def __init__(self):
        self.since = None

    def update(self, now, route_distance, on_road):
        bad = route_distance > 8. or (not on_road and route_distance > 3.5)
        self.since = (now if self.since is None else self.since) if bad else None
        return self.since is not None and now-self.since >= 1.
