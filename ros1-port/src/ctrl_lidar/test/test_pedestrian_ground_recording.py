"""Raw recorded asphalt reference must preserve the existing person classifier."""
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Header
from sensor_msgs.msg import PointField
from nav_msgs.msg import Odometry
from ctrl_ros1 import ctrl_point_cloud2 as point_cloud2
from ctrl_lidar.ctrl_obstacle_node import ObstacleNode, Track
import ctrl_lidar.ctrl_obstacle_node as detection

DATA = Path(__file__).parent/'data/pedestrian_ground_20261022'
META = json.loads((DATA/'source.json').read_text())


@pytest.fixture
def node(monkeypatch):
    rclpy.init()
    clock = [0.]
    monkeypatch.setattr(detection, 'perf_counter', lambda: clock[0])
    node = ObstacleNode(parameter_overrides=[Parameter('use_tf', value=False),
        Parameter('link_set', value=str(Path(__file__).parents[2]/'ctrl_bringup/maps/link_set.json'))])
    node.mission_name = 'PEDESTRIAN'
    node.mission_received_at = 0.
    yield node, clock
    node.destroy_node()
    rclpy.shutdown()


def crossing_detections(node, clock):
    last_pose = -1.
    for row in META['clouds']:
        path = DATA/row['file']
        assert hashlib.sha256(path.read_bytes()).hexdigest() == row['sha256']
        with np.load(path, allow_pickle=False) as data:
            raw = data['points']
        header = Header(frame_id=row['frame'])
        header.stamp.sec, header.stamp.nanosec = row['stamp']
        fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate(row['fields'])]
        message = point_cloud2.create_cloud(header, fields, raw)
        before = raw.copy()
        for pose in row['odometry']:
            if pose['record_time'] <= last_pose:
                continue
            message_pose = Odometry()
            message_pose.header.stamp.sec, message_pose.header.stamp.nanosec = pose['stamp']
            for axis, value in zip('xyz', pose['position']):
                setattr(message_pose.pose.pose.position, axis, value)
            for axis, value in zip('xyzw', pose['rotation']):
                setattr(message_pose.pose.pose.orientation, axis, value)
            message_pose.twist.twist.linear.x = pose['speed']
            node.on_odom(message_pose)
            last_pose = pose['record_time']
        node.ego_s = row['ego_s']
        clock[0] = .1
        detections = node.detect(message, row['record_time'])
        np.testing.assert_array_equal(raw, before)
        # Same crossing body in the recorded base frame; road background is retained.
        x, y = row['recorded_center_base_xy']
        target = min(detections, key=lambda d: (d['x']-x)**2 + (d['y']-y)**2)
        np.testing.assert_allclose([target['x'], target['y']], [x,y], atol=1e-5)
        yield row, target


def test_recorded_person_confirms_without_changing_shape_thresholds(node):
    node, clock = node
    track = Track(1, 5)
    for row, observed in crossing_detections(node, clock):
        features = observed['ped']
        assert node.PED_TOP[0] <= features['top'] <= node.PED_TOP[1]
        assert features['mid'] >= node.PED_MID_SHARE
        track.ped = features
        track.x = observed['x']
        node.pedestrian_vote(track)
    assert track.ped_confirmed


def test_same_recording_without_fresh_mission_keeps_existing_fallback(node):
    node, clock = node
    node.mission_received_at = -2.
    features = [observed['ped'] for _, observed in crossing_detections(node, clock)]
    assert any(f['top'] > node.PED_TOP[1] for f in features)
    assert any(f['mid'] < node.PED_MID_SHARE for f in features)
