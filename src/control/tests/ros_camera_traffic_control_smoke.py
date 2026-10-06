#!/usr/bin/env python3
"""Real controller + stamped camera inputs on a private ROS master; no vehicle sender."""
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import xmlrpc.client


def main():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:{}'.format(port)
    os.environ['ROS_IP'] = '127.0.0.1'
    os.environ.pop('ROS_HOSTNAME', None)
    logdir = Path(tempfile.mkdtemp(prefix='scmc-camera-control-ros-'))
    os.environ['ROS_LOG_DIR'] = str(logdir)
    processes, logs = [], []

    def start(args):
        log = open(logdir/'process{}.log'.format(len(processes)), 'w')
        process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process); logs.append(log)
        return process

    try:
        start(['roscore', '-p', str(port)])
        for _ in range(60):
            try:
                if xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/camera_control_test')[0] == 1:
                    break
            except OSError:
                pass
            time.sleep(.1)
        else:
            raise RuntimeError('isolated master did not start')

        import rospy
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Path as RosPath
        from morai_msgs.msg import GPSMessage
        from simul_msgs.msg import (CameraTrafficMission, ControlCmd, LocalPlan, StopLine,
                                    TrafficLightTrack, VehicleStatus)
        from std_msgs.msg import Bool, String
        rospy.init_node('camera_control_smoke', anonymous=True, disable_signals=True)
        controller = start(['roslaunch', 'control', 'control.launch',
                            'enable_camera_traffic_control:=true', 'enable_local_planner:=true'])
        latest = {}
        def receive(msg, name): latest[name] = (time.monotonic(), msg)
        subscriptions = [rospy.Subscriber(topic, kind, receive, callback_args=name, queue_size=1)
                         for topic, kind, name in (
                             ('/control_cmd', ControlCmd, 'cmd'),
                             ('/traffic_mission_state', String, 'state'))]
        publishers = {name: rospy.Publisher('/'+name, kind, queue_size=1) for name, kind in (
            ('current_pose', PoseStamped), ('global_path', RosPath), ('control_path', RosPath),
            ('vehicle_status', VehicleStatus), ('gps', GPSMessage), ('local_plan', LocalPlan),
            ('traffic_camera_mission', CameraTrafficMission), ('traffic_light/tracked', TrafficLightTrack),
            ('stop_line/detection', StopLine), ('traffic_camera_mission/reset', Bool))}
        options = dict(active=False, mission=1, track=7, velocity=0., signal='4red', observed=True,
                       held=False, line=False, y=.56, enabled=False, gps=True, short_path=False,
                       planner_stop=False, metric=False, distance=20., mission_age=0., signal_age=0.,
                       line_age=0., publish_vehicle=True, line_state='DETECTED', signal_track=7,
                       publish_mission=True, target_visible=False, filtered_signal=None)
        frozen_signal_stamp = None
        def inputs():
            now = rospy.Time.now()
            pose = PoseStamped()
            pose.header.frame_id, pose.header.stamp = 'map', now
            pose.pose.orientation.w = 1.
            publishers['current_pose'].publish(pose)
            path = RosPath(); path.header = pose.header
            for i in range(100):
                point = PoseStamped(); point.header = pose.header
                point.pose.position.x, point.pose.position.y = i*.5, i*.02
                point.pose.orientation.z, point.pose.orientation.w = math.sin(.02), math.cos(.02)
                path.poses.append(point)
            publishers['global_path'].publish(path)
            if options['short_path']: path.poses = path.poses[:3]
            publishers['control_path'].publish(path)
            if options['publish_vehicle']:
                publishers['vehicle_status'].publish(VehicleStatus(vel_x=options['velocity'], yaw=0.))
            publishers['gps'].publish(GPSMessage(latitude=37. if options['gps'] else 0.,
                                                longitude=127. if options['gps'] else 0.))
            planner = LocalPlan(); planner.header = pose.header
            planner.active = planner.stop = options['planner_stop']
            publishers['local_plan'].publish(planner)
            mission = CameraTrafficMission()
            mission.header.stamp = now-rospy.Duration(options['mission_age'])
            mission.active, mission.mission_id, mission.target_track_id = options['active'], options['mission'], options['track']
            mission.phase = 'ACTIVE' if options['active'] else 'IDLE'
            if options['publish_mission']: publishers['traffic_camera_mission'].publish(mission)
            track = TrafficLightTrack(); track.header = pose.header
            track.last_detection_stamp = frozen_signal_stamp or now-rospy.Duration(options['signal_age'])
            track.track_id = options['signal_track']
            track.raw_label = options['signal']
            track.label = options['filtered_signal'] or options['signal']
            track.signal_valid = track.label != 'UNKNOWN'
            track.observed, track.held = options['observed'], options['held']
            track.bbox_valid = options['target_visible']
            track.xmin, track.ymin, track.xmax, track.ymax = .49, .2, .51, .22
            track.width_ratio = track.raw_width_ratio = .02
            track.confidence = .9
            track.signal_age = max(0., (now-track.last_detection_stamp).to_sec())
            publishers['traffic_light/tracked'].publish(track)
            line = StopLine(); line.header.stamp = now-rospy.Duration(options['line_age'])
            line.enabled, line.detected = options['enabled'], options['line']
            line.image_y_ratio, line.distance_valid, line.distance_m = options['y'], options['metric'], options['distance']
            line.state = options['line_state'] if options['enabled'] else 'DISABLED'
            publishers['stop_line/detection'].publish(line)

        checks = 0
        def check(name, predicate, minimum=.4, timeout=7.):
            nonlocal checks
            began = time.monotonic()
            while time.monotonic()-began < timeout:
                inputs(); time.sleep(.04)
                if controller.poll() is not None:
                    raise AssertionError('controller exited; see '+str(logdir))
                if time.monotonic()-began >= minimum and all(
                        key in latest and latest[key][0] > began for key in ('cmd', 'state')):
                    if predicate(latest['cmd'][1], latest['state'][1].data):
                        checks += 1; print('PASS:', name, flush=True); return
            raise AssertionError('{} failed: {}'.format(name, latest))

        stopped = lambda cmd, state: cmd.accel == 0 and cmd.brake == 1
        moving = lambda cmd, state: cmd.accel > 0 and cmd.brake == 0
        def reset():
            nonlocal frozen_signal_stamp
            frozen_signal_stamp = None
            options.update(active=False, velocity=0., enabled=False, line=False, signal='4red',
                           observed=True, held=False, gps=True, short_path=False, planner_stop=False,
                           metric=False, mission_age=0., signal_age=0., line_age=0., publish_vehicle=True,
                           line_state='DETECTED', signal_track=options['track'], publish_mission=True,
                           target_visible=False, filtered_signal=None)
            for _ in range(3):
                publishers['traffic_camera_mission/reset'].publish(Bool(data=True))
                inputs(); time.sleep(.05)
            check('reset returns to global route', lambda cmd, state: state == 'M0:IDLE' and moving(cmd, state))

        check('idle follows global route with steering', lambda cmd, state: state == 'M0:IDLE' and moving(cmd, state) and abs(cmd.steering) > .001)
        options.update(target_visible=True, velocity=60., filtered_signal='UNKNOWN')
        check('first traffic box before mission entry brakes from 60 toward 40', lambda cmd, state: state == 'M0:PRE_APPROACH' and cmd.accel == 0 and cmd.brake > 0)
        options['velocity'] = 40.
        check('early detection prevents throttle at 40', lambda cmd, state: state == 'M0:PRE_APPROACH' and cmd.accel == 0)
        options.update(target_visible=False, observed=False, held=True, velocity=0.)
        check('brief missing box retains early 40 cap', lambda cmd, state: state == 'M0:PRE_APPROACH', minimum=.3)
        check('early cap expires after two seconds without real observation', lambda cmd, state: state == 'M0:IDLE' and moving(cmd, state), minimum=2.2)
        options.update(observed=True, held=False, signal='garbage', target_visible=True)
        check('unrecognized box class cannot start early cap', lambda cmd, state: state == 'M0:IDLE' and moving(cmd, state))
        options.update(signal='4red', target_visible=False, filtered_signal=None)
        options.update(active=True, enabled=True, velocity=35.)
        check('mission enters slow stop-line search', lambda cmd, state: state == 'M1:SEARCH_LINE' and cmd.accel == 0 and cmd.brake > 0)
        options.update(line=True, y=.56, velocity=20.)
        check('red approaches line and retains path steering', lambda cmd, state: state == 'M1:APPROACH_IMAGE' and cmd.brake > 0 and abs(cmd.steering) > .001)
        options.update(y=.76, velocity=0.)
        for label in ('4red', '4yellow', 'UNKNOWN', '4redleft'):
            options['signal'] = label
            check(label+' waits at line', lambda cmd, state: state == 'M1:WAIT_SIGNAL' and stopped(cmd, state))
        options.update(signal='4green', observed=False, held=True)
        check('held green cannot initiate departure', stopped)
        options.update(observed=True, held=False, signal_track=8)
        check('another signal track cannot release stop', stopped)
        options['signal_track'] = 7
        frozen_signal_stamp = rospy.Time.now()
        check('repeated green source stamp cannot release stop', stopped)
        frozen_signal_stamp = None
        options['line_state'] = 'STALE_IMAGE'
        check('stop camera fault blocks green departure', lambda cmd, state: state == 'M1:STOP_CAMERA_STALE' and stopped(cmd, state))
        options.update(signal='4red', line_state='DETECTED')
        check('red recovery remains stopped', stopped)
        options.update(signal='4green', planner_stop=True)
        check('blocked path cannot commit a green departure', lambda cmd, state: state == 'M1:WAIT_PATH' and stopped(cmd, state))
        options.update(signal='4red', planner_stop=False)
        check('path recovery on red stays stopped', lambda cmd, state: state == 'M1:WAIT_SIGNAL' and stopped(cmd, state))
        options['signal'] = '4green'
        check('three observed straight greens depart', lambda cmd, state: state == 'M1:CROSSING' and moving(cmd, state))
        options.update(velocity=45.)
        check('crossing above 40 brakes instead of accelerating', lambda cmd, state: state == 'M1:CROSSING' and cmd.accel == 0 and cmd.brake > 0)
        options['velocity'] = 40.
        check('40 km/h ceiling prevents throttle', lambda cmd, state: state == 'M1:CROSSING' and cmd.accel == 0)
        options.update(velocity=0., signal='UNKNOWN', line_state='STALE_IMAGE', mission_age=2.)
        check('committed crossing survives lost camera signal', lambda cmd, state: state == 'M1:CROSSING' and moving(cmd, state))
        options.update(planner_stop=True)
        check('green does not override local planner stop', stopped)
        options.update(planner_stop=False, short_path=True)
        check('green does not override path-end stop', stopped)
        options.update(short_path=False, active=False, mission_age=0.)
        check('camera exit after crossing restores normal route', lambda cmd, state: state == 'M0:IDLE' and moving(cmd, state))
        options.update(mission=2, track=9, signal_track=9, active=True, enabled=True, line=True,
                       signal='4red', y=.76, line_state='DETECTED')
        check('next mission needs its own green', lambda cmd, state: state == 'M2:WAIT_SIGNAL' and stopped(cmd, state))
        options['active'] = False
        check('camera-edge exit before departure cannot release red stop', lambda cmd, state: state == 'M2:MISSION_CHANGED_BEFORE_CROSSING' and stopped(cmd, state))
        reset()
        options.update(active=True, enabled=True, line=True, y=.76, signal='4red', gps=False)
        check('GPS shadow still obeys camera red stop', lambda cmd, state: state == 'M2:WAIT_SIGNAL' and stopped(cmd, state))
        options['signal'] = '4greenleft'
        check('left green also departs in GPS shadow', lambda cmd, state: state == 'M2:CROSSING' and moving(cmd, state))
        options['velocity'] = 42.
        check('GPS shadow crossing enforces 40 km/h', lambda cmd, state: cmd.accel == 0 and cmd.brake > 0)
        reset()
        options.update(active=True, enabled=True, line=True, y=.57, velocity=0.)
        check('new mission approaches far line', lambda cmd, state: state == 'M2:APPROACH_IMAGE' and moving(cmd, state))
        options['line'] = False
        check('lost stop line expires held position', lambda cmd, state: state == 'M2:STOP_LINE_LOST' and stopped(cmd, state), minimum=.6)
        options.update(line=True, y=.76, signal='4green', signal_age=2.)
        check('old green with fresh publication cannot release wait', stopped)
        options.update(publish_mission=False)
        check('mission stream loss brakes', lambda cmd, state: state == 'M2:MISSION_STALE' and stopped(cmd, state), minimum=1.)
        options.update(publish_mission=True, publish_vehicle=False)
        check('vehicle status loss brakes', lambda cmd, state: state == 'M2:VEHICLE_STALE' and stopped(cmd, state), minimum=.7)
        reset()
        options.update(active=True, enabled=True, line=True, metric=True, distance=25., y=.78, velocity=50.)
        check('metric approach uses calibrated distance and respects ceiling', lambda cmd, state: state == 'M2:APPROACH_METRIC' and cmd.accel == 0 and cmd.brake > 0)
        options.update(distance=2., velocity=0.)
        check('metric stop respects bumper margin', lambda cmd, state: state == 'M2:WAIT_SIGNAL' and stopped(cmd, state))
        print('Camera traffic ROS integration: {} checks passed. Logs: {}'.format(checks, logdir), flush=True)
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL); process.wait()
        for log in logs: log.close()


if __name__ == '__main__':
    main()
