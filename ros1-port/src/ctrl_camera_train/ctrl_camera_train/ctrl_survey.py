"""Interactive baseline route survey. No old stop-line table is loaded."""
import csv
import json
import math
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import tempfile
import time
from datetime import datetime

import ctrl_ros1 as rclpy
from ctrl_ros1.ctrl_node import Node
from ctrl_ros1.ctrl_qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64
from ctrl_interfaces.msg import ControlCommand, VehicleStatus


def ready_to_save(samples, now, stopped_since):
    required = ('odom', 's', 'status', 'rgb', 'sem')
    if any(k not in samples or now - samples[k][0] > .5 for k in required):
        return False
    if stopped_since is None or now - stopped_since < 1.:
        return False
    a, b = samples['rgb'][1], samples['sem'][1]
    stamp = lambda m: m.header.stamp.secs + m.header.stamp.nsecs / 1e9
    return abs(stamp(a) - stamp(b)) <= .005


class Survey(Node):
    def __init__(self, out):
        super().__init__('survey_points')
        self.out = out
        self.samples = {}
        self.running = False
        self.pending = None
        self.stopped_since = None
        self.count = 0
        self.pub = self.create_publisher(ControlCommand, '/control/request', 1)
        for key, cls, topic in [
            ('odom', Odometry, '/localization/odometry'),
            ('s', Float64, '/localization/s'),
            ('status', VehicleStatus, '/vehicle/status'),
            ('rgb', CompressedImage, '/sensors/camera/tele/compressed'),
            ('sem', CompressedImage, '/sensors/camera/semantic/compressed'),
            ('command', ControlCommand, '/survey/baseline_request')]:
            self.create_subscription(cls, topic, lambda msg, k=key: self.receive(k, msg), qos_profile_sensor_data)
        self.create_timer(.02, self.tick)

    def receive(self, key, msg):
        self.samples[key] = (time.monotonic(), msg)

    def tick(self):
        now = time.monotonic()
        status = self.samples.get('status')
        stationary = status and now-status[0] <= .5 and math.isfinite(status[1].speed_mps) and abs(status[1].speed_mps) < .05
        if not stationary:
            self.stopped_since = None
        elif self.stopped_since is None:
            self.stopped_since = now
        cmd = self.samples.get('command')
        msg = ControlCommand()
        if self.running and cmd and now-cmd[0] < .2:
            msg.accel, msg.brake, msg.steering_rad = cmd[1].accel, cmd[1].brake, cmd[1].steering_rad
        else:
            msg.brake = 1.
            if cmd and now-cmd[0] < .2:
                msg.steering_rad = cmd[1].steering_rad
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = 'SURVEY_DRIVE' if self.running else 'SURVEY_HOLD'
        self.pub.publish(msg)
        if self.pending and ready_to_save(self.samples, now, self.stopped_since):
            self.save()

    def save(self):
        odom = self.samples['odom'][1]
        p, q = odom.pose.pose.position, odom.pose.pose.orientation
        s = self.samples['s'][1].data
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        if not all(math.isfinite(v) for v in (p.x, p.y, s, yaw)) or odom.pose.covariance[0] != 0:
            return
        self.count += 1
        name = f'{self.count:04d}_{self.pending}'
        destination = self.out / name
        destination.mkdir()
        info = dict(kind=self.pending, timestamp=datetime.now().isoformat(), x=p.x, y=p.y,
                    s=s, yaw_rad=yaw, coordinate_reference='rear_axle',
                    note='stopline_reference is a vehicle pose, not a measured line coordinate')
        for key in ('rgb', 'sem'):
            msg = self.samples[key][1]
            data = bytes(msg.data)
            ext = '.png' if data.startswith(b'\x89PNG') else '.jpg'
            (destination / (key+ext)).write_bytes(data)
            info[key+'_stamp'] = dict(sec=msg.header.stamp.secs, nanosec=msg.header.stamp.nsecs)
        (destination/'point.json').write_text(json.dumps(info, ensure_ascii=False, indent=2))
        with (self.out/'points.csv').open('a', newline='') as f:
            writer = csv.writer(f)
            if self.count == 1:
                writer.writerow(['id', 'kind', 'x', 'y', 's', 'yaw_rad', 'reference'])
            writer.writerow([name, self.pending, p.x, p.y, s, yaw, 'rear_axle'])
        print(f'저장: {destination} | s={s:.2f}m. 정차 유지 중.', flush=True)
        self.pending = None


def main():
    if not sys.stdin.isatty():
        raise SystemExit('직접 입력할 수 있는 터미널에서 실행하세요.')
    import fcntl
    lock = open('/tmp/foscar_survey_points.lock', 'w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit('위치 기록 프로그램이 이미 실행 중입니다.')
    # Refuse to compete with existing ROS control publishers.
    repo = Path(__file__).resolve().parents[3]
    out = Path.home()/'foscar_logs/survey'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    out.mkdir(parents=True)
    rclpy.init()
    node = Survey(out)
    child = None
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=.05)
    if node.count_publishers('/control/request') > 1 or node.count_publishers('/survey/baseline_request') > 0:
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit('기존 주행/수집 프로그램을 Ctrl+C로 종료한 뒤 실행하세요.')
    print(f'저장 폴더: {out}\ng Enter: 자동 주행 | Enter: 정차 후 촬영 위치 저장 | l Enter: 정지선 기준점 | q Enter: 종료\n카메라 창 자동 실행. 조작은 이 터미널에서 입력하세요. 창을 닫으면 전체 종료합니다.\n초기 정차. 정지선 기록은 차량 후륜축 위치이며 실제 선 좌표와 다릅니다.', flush=True)
    try:
        with tempfile.TemporaryDirectory(prefix='survey_') as tmp:
            # Remap only the follower output: Survey is the sole bridge command source.
            launch = Path(tmp)/'survey.launch.py'
            launch.write_text('''from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, EmitEvent, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

def generate_launch_description():
    nodes = ''' + repr([]) + '''
''' + '\n'.join(
                '    nodes.append(Node(package='+repr(pkg)+', executable='+repr(exe)+', name='+repr(name)+
                ', parameters='+repr(params)+', remappings='+repr(remaps)+', output="screen"))'
                for pkg, exe, name, params, remaps in [
                    ('ctrl_control','sensor_bridge','sensor_bridge',[str(repo/'src/ctrl_bringup/config/low_speed.yaml'),{'drive':True}],[]),
                    ('ctrl_control','localization','localization',[str(repo/'src/ctrl_bringup/config/low_speed.yaml'),{'path_file':str(repo/'src/ctrl_bringup/routes/global_path.txt'),'spawn_file':str(repo/'src/ctrl_bringup/routes/spawn_path.txt')}],[]),
                    ('ctrl_control','path_follower','path_follower',[str(repo/'src/ctrl_bringup/config/low_speed.yaml'),{'path_file':str(repo/'src/ctrl_bringup/routes/global_path.txt'),'spawn_file':str(repo/'src/ctrl_bringup/routes/spawn_path.txt'),'speed_kmh':5.,'continuous':True}], [('/control/request','/survey/baseline_request')]),
                    ('ctrl_camera','camera_bridge','camera_bridge',[str(repo/'src/ctrl_camera_train/config/record.yaml')],[]),
                    ('ctrl_camera','camera_bridge','camera_bridge_semantic',[str(repo/'src/ctrl_camera_train/config/record.yaml')],[])]) + '''
    nodes.append(ExecuteProcess(cmd=VIEWER_COMMAND, output='screen'))
    guards = [RegisterEventHandler(OnProcessExit(target_action=n, on_exit=[EmitEvent(event=Shutdown(reason='survey node exited'))])) for n in nodes]
    return LaunchDescription(guards + nodes)
'''.replace('VIEWER_COMMAND', repr([sys.executable, str(repo/'tools/survey_view.py')])))
            with (out/'launch.log').open('w') as log:
                child = subprocess.Popen(['ros2','launch',str(launch)], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                while rclpy.ok() and child.poll() is None:
                    rclpy.spin_once(node, timeout_sec=.02)
                    if select.select([sys.stdin], [], [], 0)[0]:
                        line = sys.stdin.readline()
                        if not line or line.strip() == 'q':
                            break
                        key = line.strip()
                        if key == 'g':
                            node.pending = None
                            node.running = True
                            print('자동 경로 주행 요청 (5 km/h). 정차는 Enter.', flush=True)
                        elif key in ('', 'l'):
                            node.running = False
                            node.pending = 'capture' if key == '' else 'stopline_reference'
                            node.stopped_since = None
                            print('정차 요청. 정지 1초 + 최신 GPS 위치/동기 영상 확보 후 저장합니다.', flush=True)
                        else:
                            print('g / Enter / l / q 중 입력하세요.', flush=True)
                node.running = False
                node.pending = None
                until = time.monotonic()+2
                while time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=.02)
    except KeyboardInterrupt:
        pass
    finally:
        if child and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                child.wait(timeout=5)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(f'종료. 저장 위치: {out}', flush=True)

if __name__ == '__main__':
    main()
