#!/usr/bin/env python3
"""Isolated synthetic ROS integration test. Never starts MORAI UDP or a sensor driver.
Run after catkin_make and sourcing devel/setup.bash.
"""
import os
import sys
import signal
import socket
import subprocess
import tempfile
import time
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[2]
with socket.socket() as sock:
    sock.bind(('127.0.0.1', 0))
    port = sock.getsockname()[1]
os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % port
os.environ['ROS_IP'] = '127.0.0.1'
os.environ.pop('ROS_HOSTNAME', None)
os.environ['ROS_LOG_DIR'] = tempfile.mkdtemp(prefix='scmc-local-ros-')
os.environ['OPENBLAS_NUM_THREADS'] = '1'
processes, logs = [], []


def start(args):
    log = open(os.path.join(os.environ['ROS_LOG_DIR'], 'process%d.log' % len(processes)), 'w')
    p = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    processes.append(p)
    logs.append(log)
    return p


def stop(p):
    if p.poll() is None:
        os.killpg(p.pid, signal.SIGINT)
        try:
            p.wait(timeout=4)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
            p.wait()


try:
    start(['roscore', '-p', str(port)])
    import xmlrpc.client
    for _ in range(60):
        try:
            if xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/smoke')[0] == 1:
                break
        except OSError:
            pass
        time.sleep(.1)
    else:
        raise RuntimeError('test ROS master did not start')
    import rospy
    from nav_msgs.msg import Path as RosPath
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Header
    from sensor_msgs import point_cloud2
    from sensor_msgs.msg import PointCloud2
    from morai_msgs.msg import GPSMessage
    from simul_msgs.msg import VehicleStatus, ControlCmd, LocalPlan
    from lidar_object_detection.msg import ObjectInfo
    rospy.init_node('local_planner_smoke', anonymous=True)
    config = yaml.safe_load((ROOT / 'planning/config/local_planner.yaml').read_text())
    config['static_obstacle_avoidance_planner']['static_zones'] = [dict(xmin=-1., xmax=30., ymin=-4., ymax=4.)]
    config['static_obstacle_avoidance_planner']['dynamic_zones'] = [dict(xmin=50., xmax=70., ymin=-4., ymax=4.)]
    config['viz_planner']['lidar_xyz_rpy'] = [0., 0., 1., 0., 0., 0.]  # SYNTHETIC fixture only
    for node, params in config.items():
        rospy.set_param('/' + node, params)
    rospy.set_param('/control/enable_local_planner', True)
    start(['rosrun', 'planning', 'VizPlanner.py', '__name:=viz_planner'])
    planner = start(['rosrun', 'planning', 'StaticObstacleAvoidancePlanner.py', '__name:=static_obstacle_avoidance_planner'])
    start(['rosrun', 'control', 'control', '__name:=control'])
    start(['rosrun', 'control', 'control', '__name:=baseline_control', '/control_cmd:=/baseline_control_cmd', '/curr_idx:=/baseline_idx'])
    latest = {}
    def receive(msg, key):
        latest[key] = msg
    subscriptions = [rospy.Subscriber(topic, kind, receive, callback_args=key) for topic, kind, key in
                     [('/local_plan', LocalPlan, 'plan'), ('/control_cmd', ControlCmd, 'command'),
                      ('/baseline_control_cmd', ControlCmd, 'baseline'), ('/obstacle_info_static', ObjectInfo, 'obstacles')]]
    publishers = {name: rospy.Publisher('/' + name, kind, queue_size=1) for name, kind in
                  [('global_path', RosPath), ('control_path', RosPath), ('current_pose', PoseStamped),
                   ('vehicle_status', VehicleStatus), ('gps', GPSMessage),
                   ('obstacle_info_static', ObjectInfo), ('velodyne_points', PointCloud2)]}
    vehicle = [0., 0.]
    objects = [(12., 0., 1.)]
    send_obstacles = True
    obstacle_frame = 'velodyne'
    path = RosPath()
    path.header.frame_id = 'map'
    for i in range(401):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = i * .5
        pose.pose.orientation.w = 1.
        path.poses.append(pose)

    last_pose_time = [0.]

    def publish_inputs():
        now = rospy.Time.now()
        path.header.stamp = now
        publishers['global_path'].publish(path)
        reference = RosPath()
        reference.header = path.header
        idx = max(0, int(vehicle[0] / .5))
        reference.poses = path.poses[idx:idx + 50]
        publishers['control_path'].publish(reference)
        pose = PoseStamped()
        pose.header.frame_id, pose.header.stamp = 'map', now
        pose.pose.position.x, pose.pose.position.y = vehicle
        pose.pose.orientation.w = 1.
        if time.monotonic() - last_pose_time[0] >= .2:  # slower pose stream than LiDAR
            publishers['current_pose'].publish(pose)
            last_pose_time[0] = time.monotonic()
        publishers['vehicle_status'].publish(VehicleStatus(vel_x=0., yaw=0.))
        publishers['gps'].publish(GPSMessage(latitude=37., longitude=127.))
        if send_obstacles:
            msg = ObjectInfo()
            msg.header = Header(stamp=now, frame_id=obstacle_frame)
            msg.objectCounts = len(objects)
            for i, (x, y, size) in enumerate(objects):
                msg.centerX[i], msg.centerY[i], msg.centerZ[i] = x, y, -.5
                msg.lengthX[i] = msg.lengthY[i] = size
                msg.lengthZ[i] = .5
            publishers['obstacle_info_static'].publish(msg)

    def check(name, predicate, timeout=7, minimum=.8):
        begun = time.monotonic()
        while time.monotonic() - begun < timeout:
            publish_inputs()
            time.sleep(.03)
            if time.monotonic() - begun >= minimum and predicate():
                print('PASS:', name, flush=True)
                return
        raise AssertionError('%s failed; plan=%s command=%s' % (name, latest.get('plan'), latest.get('command')))

    check('static avoidance selects a collision-free lateral path',
          lambda: 'plan' in latest and latest['plan'].state == 'STATIC_OBSTACLE' and
          not latest['plan'].stop and max(abs(p.pose.position.y) for p in latest['plan'].path.poses) > 2)
    check('controller follows local target with acceleration and steering',
          lambda: 'command' in latest and latest['command'].accel > 0 and abs(latest['command'].steering) > .001)
    objects = [(8., 0., 12.)]
    check('all candidates blocked -> controller full brake',
          lambda: latest['plan'].stop and latest['command'].brake == 1 and latest['command'].accel == 0)
    objects = []
    check('empty detections expire obstacle memory and resume',
          lambda: not latest['plan'].stop and latest['command'].accel > 0, timeout=8, minimum=2.3)
    send_obstacles = False
    check('LiDAR timeout -> hold and full brake',
          lambda: latest['plan'].stop and 'LiDAR' in latest['plan'].state and latest['command'].brake == 1)
    send_obstacles = True
    check('LiDAR recovery -> resume', lambda: not latest['plan'].stop and latest['command'].accel > 0)
    obstacle_frame = 'unconnected_sensor'
    check('missing TF cannot be treated as an empty road', lambda: latest['plan'].stop and latest['command'].brake == 1)
    obstacle_frame = 'velodyne'
    check('TF recovery resumes planning', lambda: not latest['plan'].stop and latest['command'].accel > 0)
    vehicle[:] = [32., 1.]
    check('zone exit keeps local control while returning', lambda: latest['plan'].state == 'RETURN_TO_GLOBAL')
    vehicle[:] = [34., 0.]
    check('aligned return releases global controller', lambda: latest['plan'].state == 'NORMAL' and not latest['plan'].active)
    check('normal mode output matches unmodified controller branch',
          lambda: 'baseline' in latest and abs(latest['baseline'].accel - latest['command'].accel) < 1e-8 and
          abs(latest['baseline'].brake - latest['command'].brake) < 1e-8 and
          abs(latest['baseline'].steering - latest['command'].steering) < 1e-5)
    vehicle[:] = [55., 0.]
    objects = [(8., 0., 1.)]
    check('dynamic zone stops without lateral avoidance',
          lambda: latest['plan'].state == 'DYNAMIC_OBSTACLE: stop' and latest['command'].brake == 1)
    objects = []
    check('dynamic corridor clear resumes without lateral avoidance',
          lambda: latest['plan'].state == 'DYNAMIC_OBSTACLE: corridor_clear' and latest['command'].accel > 0,
          minimum=2.3)
    stop(planner)
    check('planner process loss -> controller full brake', lambda: latest['command'].brake == 1 and latest['command'].accel == 0,
          minimum=1.)
    # Validate actual C++ DBSCAN, including valid empty scans and acquisition metadata.
    send_obstacles = False
    start(['rosrun', 'lidar_object_detection', 'lidar_object_detection_static_node', '__name:=object_detection_static'])
    time.sleep(1)
    import itertools
    points = list(itertools.product([8., 8.1, 8.2, 8.3], [2., 2.1, 2.2, 2.3], [-1.2, -1.1, -1., -.9]))
    for empty in (False, True):
        stamp = rospy.Time.now()
        cloud = point_cloud2.create_cloud_xyz32(Header(stamp=stamp, frame_id='velodyne'), [] if empty else points)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            publishers['velodyne_points'].publish(cloud)
            time.sleep(.1)
            detected = latest.get('obstacles')
            if detected is not None and detected.header.stamp == stamp:
                assert detected.header.frame_id == 'velodyne'
                assert detected.objectCounts == (0 if empty else 1), detected.objectCounts
                break
        else:
            raise AssertionError('no timestamped detector output')
        print('PASS: DBSCAN %s scan metadata and count' % ('empty' if empty else 'clustered'), flush=True)
    print('All isolated ROS checks passed. Logs:', os.environ['ROS_LOG_DIR'], flush=True)
finally:
    for p in reversed(processes):
        stop(p)
    for log in logs:
        log.close()
