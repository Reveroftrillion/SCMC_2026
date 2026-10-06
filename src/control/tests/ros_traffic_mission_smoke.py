#!/usr/bin/env python3
"""Exercise the real control.launch with synthetic inputs on an isolated ROS master.
Never starts MORAI UDP or sends commands to a vehicle.
"""
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import xmlrpc.client

import yaml

ROOT = Path(__file__).resolve().parents[3]


def main():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:{}'.format(port)
    os.environ['ROS_IP'] = '127.0.0.1'
    os.environ.pop('ROS_HOSTNAME', None)
    logdir = tempfile.mkdtemp(prefix='scmc-traffic-ros-')
    os.environ['ROS_LOG_DIR'] = logdir
    processes, logs = [], []

    def start(args):
        log = open(os.path.join(logdir, 'process{}.log'.format(len(processes))), 'w')
        p = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(p)
        logs.append(log)
        return p

    try:
        start(['roscore', '-p', str(port)])
        for _ in range(60):
            try:
                if xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/traffic_test')[0] == 1:
                    break
            except OSError:
                pass
            time.sleep(.1)
        else:
            raise RuntimeError('isolated ROS master did not start')

        import rospy
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Path as RosPath
        from morai_msgs.msg import GPSMessage
        from simul_msgs.msg import VehicleStatus, ControlCmd
        from std_msgs.msg import String
        rospy.init_node('traffic_mission_smoke', anonymous=True)
        controller = start(['roslaunch', 'control', 'control.launch', 'enable_traffic_missions:=true'])
        missions = yaml.safe_load((ROOT/'src/control/config/traffic_missions.yaml').read_text())['traffic_missions']
        latest = {}

        def receive(msg, key):
            latest[key] = (time.monotonic(), msg)

        subscribers = [
            rospy.Subscriber('/control_cmd', ControlCmd, receive, callback_args='command'),
            rospy.Subscriber('/traffic_mission_state', String, receive, callback_args='state'),
        ]
        publishers = {name: rospy.Publisher('/'+name, kind, queue_size=1) for name, kind in [
            ('current_pose', PoseStamped), ('global_path', RosPath), ('control_path', RosPath),
            ('vehicle_status', VehicleStatus), ('gps', GPSMessage), ('traffic_light_status', String)]}
        pose_xy = [0., 0.]
        heading = 0.
        velocity = 0.
        signal_label = '4red'
        gps_valid = True
        publish_pose = True
        frame_id = 'map'

        def inputs():
            now = rospy.Time.now()
            pose = PoseStamped()
            pose.header.frame_id, pose.header.stamp = frame_id, now
            pose.pose.position.x, pose.pose.position.y = pose_xy
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(heading/2), math.cos(heading/2)
            if publish_pose:
                publishers['current_pose'].publish(pose)
            path = RosPath()
            path.header.frame_id, path.header.stamp = 'map', now
            for i in range(100):
                point = PoseStamped()
                point.header = path.header
                point.pose.position.x = pose_xy[0]+i*.5*math.cos(heading)
                point.pose.position.y = pose_xy[1]+i*.5*math.sin(heading)
                point.pose.orientation = pose.pose.orientation
                path.poses.append(point)
            publishers['global_path'].publish(path)
            publishers['control_path'].publish(path)
            publishers['vehicle_status'].publish(VehicleStatus(vel_x=velocity, yaw=math.degrees(heading)))
            publishers['gps'].publish(GPSMessage(latitude=37. if gps_valid else 0., longitude=127. if gps_valid else 0.))
            if signal_label is not None:
                publishers['traffic_light_status'].publish(String(data=signal_label))

        def check(name, predicate, minimum=.5, timeout=5.):
            began = time.monotonic()
            while time.monotonic()-began < timeout:
                inputs()
                time.sleep(.04)
                if controller.poll() is not None:
                    raise AssertionError('control.launch exited; see '+logdir)
                if time.monotonic()-began >= minimum and all(
                        key in latest and latest[key][0] > began for key in ('command', 'state')):
                    if predicate(latest['command'][1], latest['state'][1].data):
                        print('PASS:', name, flush=True)
                        return
            raise AssertionError('{} failed: {}'.format(name, latest))

        for mission in missions:
            name = mission['name']
            detect, stop, passed = mission['detect'], mission['stop'], mission['pass']
            heading = math.atan2(passed[1]-detect[1], passed[0]-detect[0])
            pose_xy[:] = detect
            velocity, signal_label = 25., '4red'
            check(name+' approach limits speed', lambda cmd, state: state == name+':APPROACH' and cmd.accel == 0 and cmd.brake > 0)
            pose_xy[:] = stop
            velocity = 0.
            for signal_label in ('4red', '4yellow', 'UNKNOWN', '4green'):
                check(name+' '+signal_label+' stops', lambda cmd, state: state == name+':STOP' and cmd.accel == 0 and cmd.brake == 1)
            signal_label = '4greenleft'
            # At the exact stop coordinate green commits crossing. Move 1m before it first.
            pose_xy[:] = [stop[0]-math.cos(heading), stop[1]-math.sin(heading)]
            check(name+' greenleft resumes', lambda cmd, state: state == name+':GO' and cmd.accel > 0 and cmd.brake == 0)
            signal_label = None
            check(name+' stale green stops', lambda cmd, state: state == name+':STOP' and cmd.brake == 1 and cmd.accel == 0, minimum=1.)
            signal_label = '4greenleft'
            check(name+' green recovery', lambda cmd, state: state == name+':GO' and cmd.accel > 0)
            gps_valid = False
            check(name+' GPS loss stops', lambda cmd, state: state == name+':INVALID_POSE' and cmd.brake == 1)
            gps_valid = True
            check(name+' GPS recovery', lambda cmd, state: state == name+':GO' and cmd.accel > 0)
            publish_pose = False
            check(name+' pose stream loss stops', lambda cmd, state: state == name+':INVALID_POSE' and cmd.brake == 1, minimum=1.)
            publish_pose = True
            frame_id = 'wrong_frame'
            check(name+' wrong pose frame stops', lambda cmd, state: state == name+':INVALID_POSE' and cmd.brake == 1)
            frame_id = 'map'
            pose_xy[:] = [stop[0]+math.cos(heading), stop[1]+math.sin(heading)]
            check(name+' crosses on GO', lambda cmd, state: state == name+':CROSSING' and cmd.accel > 0)
            signal_label = '4red'
            check(name+' completes committed crossing', lambda cmd, state: state == name+':CROSSING' and cmd.accel > 0)
            pose_xy[:] = [detect[0]+1.12*(passed[0]-detect[0]), detect[1]+1.12*(passed[1]-detect[1])]
            check(name+' advances only after pass margin', lambda cmd, state: not state.startswith(name+':'))

        print('ROS traffic integration: ALL PASSED. Logs:', logdir, flush=True)
    finally:
        for p in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
                try:
                    p.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid, signal.SIGKILL)
                    p.wait()
        for log in logs:
            log.close()


if __name__ == '__main__':
    main()
