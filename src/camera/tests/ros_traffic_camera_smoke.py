#!/usr/bin/env python3
"""Isolated ROS transport test; controlled YOLO boxes, real stop-line processing."""
import collections
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import xmlrpc.client

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'scripts'))


def fake_node():
    import rospy
    names = dict(enumerate(['4red', '4redleft', '4yellow', '4redyellow', '4greenleft',
                            '4green', '3red', '3redleft', '3redyellow']))

    class FakeModel:
        def __init__(self, _weights): self.names = names
        def to(self, _device): return self
        def predict(self, *_args, **_kwargs):
            boxes = [SimpleNamespace(cls=[value['cls']], conf=[.9], xyxy=[value['box']])
                     for value in rospy.get_param('/test/boxes', [])]
            return [SimpleNamespace(boxes=boxes, names=self.names)]

    sys.modules['ultralytics'] = SimpleNamespace(YOLO=FakeModel)
    import traffic_yolo
    raise SystemExit(traffic_yolo.main())


def main():
    with socket.socket() as listener:
        listener.bind(('127.0.0.1', 0))
        port = listener.getsockname()[1]
    directory = Path(tempfile.mkdtemp(prefix='scmc-traffic-camera-ros-'))
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:{}'.format(port)
    os.environ['ROS_IP'] = '127.0.0.1'
    os.environ['ROS_LOG_DIR'] = str(directory)
    children, logs = [], []
    passed = []

    def start(command):
        log = open(str(directory/'process{}.log'.format(len(children))), 'w')
        logs.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        children.append(process)
        return process

    def check(condition, description):
        if not condition: raise AssertionError(description)
        passed.append(description)
        print('PASS:', description, flush=True)

    try:
        start(['roscore', '-p', str(port)])
        master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        ready = False
        deadline = time.monotonic()+8
        while time.monotonic() < deadline:
            try:
                ready = master.getPid('/test')[0] == 1
                if ready: break
            except OSError: pass
            time.sleep(.05)
        if not ready: raise RuntimeError('Isolated ROS master failed to start')
        import cv2
        import numpy as np
        import rospy
        import yaml
        from sensor_msgs.msg import CompressedImage
        from std_msgs.msg import Bool, String
        from simul_msgs.msg import CameraTrafficMission, StopLine, TrafficLightTrack
        rospy.init_node('traffic_camera_smoke', anonymous=True, disable_signals=True)
        config = yaml.safe_load((PACKAGE/'config/traffic_camera.yaml').read_text())
        weights = directory/'dummy.pt'
        weights.write_bytes(b'test')
        config['weights_path'] = str(weights)
        rospy.set_param('/yolov8_node', config)
        rospy.set_param('/test/boxes', [])
        start([sys.executable, str(Path(__file__).resolve()), '--fake-yolo-node',
               '__name:=yolov8_node', '/camera/traffic/image/compressed:=/test/traffic/image'])
        start(['roslaunch', 'camera', 'stop_line.launch', 'require_mission_active:=false',
               'image_topic:=/test/front/image'])
        latest = {}
        def receive(message, name): latest[name] = message
        subscriptions = [rospy.Subscriber(topic, kind, receive, callback_args=name, queue_size=1)
                         for topic, kind, name in (
                             ('/traffic_light/tracked', TrafficLightTrack, 'track'),
                             ('/traffic_camera_mission', CameraTrafficMission, 'mission'),
                             ('/traffic_light_status', String, 'status'),
                             ('/traffic_mission_active', Bool, 'active'),
                             ('/stop_line/detection', StopLine, 'stop'))]
        front_pub = rospy.Publisher('/test/traffic/image', CompressedImage, queue_size=1)
        stop_pub = rospy.Publisher('/test/front/image', CompressedImage, queue_size=1)
        reset_pub = rospy.Publisher('/traffic_camera_mission/reset', Bool, queue_size=1)
        front_data = cv2.imencode('.jpg', np.zeros((720, 1280, 3), np.uint8))[1].tobytes()
        road_frame = np.full((720, 1280, 3), 40, dtype=np.uint8)
        cv2.rectangle(road_frame, (100, 460), (1180, 468), (255, 255, 255), -1)
        stop_data = cv2.imencode('.jpg', road_frame)[1].tobytes()

        def pump(seconds, front=True):
            deadline = time.monotonic()+seconds
            while time.monotonic() < deadline:
                for publisher, data, frame in ((front_pub, front_data, 'traffic'),
                                                (stop_pub, stop_data, 'front')):
                    if publisher is front_pub and not front: continue
                    message = CompressedImage()
                    message.header.stamp = rospy.Time.now()
                    message.header.frame_id, message.format, message.data = frame, 'jpeg', data
                    publisher.publish(message)
                time.sleep(.05)

        def boxes(width=.05, y=.25, cls=0):
            rospy.set_param('/test/boxes', [{'cls': cls, 'box': [(.5-width/2)*1280,
                             (y-.01)*720, (.5+width/2)*1280, (y+.01)*720]}])

        deadline = time.monotonic()+8
        while time.monotonic() < deadline and (front_pub.get_num_connections() == 0 or
                                               stop_pub.get_num_connections() == 0 or len(latest) < 5):
            pump(.1)
        check(len(latest) == 5, 'all output topics connected')
        boxes(.025)
        pump(.8)
        check(not latest['active'].data and latest['stop'].detected, 'front stop-line detector runs before mission entry')
        check(latest['track'].header.frame_id == 'traffic' and latest['stop'].header.frame_id == 'front',
              'traffic and stop-line outputs use different intended camera sources')
        boxes(.05)
        pump(.8)
        check(latest['active'].data and latest['mission'].phase == 'ACTIVE', 'confirmed size enters mission')
        check(latest['stop'].detected, 'mission entry keeps front stop-line detector ready')
        rospy.set_param('/test/boxes', [])
        pump(.15)
        check(latest['track'].held and latest['status'].data == '4red', 'brief miss is explicitly held')
        pump(.4)
        check(latest['status'].data == 'UNKNOWN' and latest['active'].data, 'long miss expires colour and keeps mission active')
        boxes(.05, cls=4)
        pump(.3)
        check(latest['status'].data == '4greenleft', 'new GO needs observed confirmation')
        boxes(.05, cls=0)
        pump(.08)
        check(latest['status'].data == '4red', 'red cancels previous GO immediately')
        for y in (.23, .18, .13, .08, .035, .02):
            boxes(.09, y)
            pump(.18)
        check(latest['mission'].exit_armed, 'growth and repeated edge observations arm exit')
        pump(.9, front=False)
        check(latest['mission'].phase == 'ACTIVE_STALE' and latest['active'].data,
              'camera outage does not finish mission')
        check(not latest['track'].bbox_valid and latest['status'].data == 'UNKNOWN', 'camera outage invalidates signal and box')
        rospy.set_param('/test/boxes', [])
        pump(.6)
        check(latest['active'].data and not latest['mission'].exit_armed, 'outage cancels exit evidence')
        boxes(.09, .02)
        pump(.5)
        check(latest['mission'].exit_armed, 'fresh target observations rearm exit')
        rospy.set_param('/test/boxes', [])
        pump(1.3)
        check(not latest['active'].data and latest['mission'].phase == 'COOLDOWN', 'healthy camera target exit finishes mission')
        check(latest['stop'].detected, 'mission end preserves continuous front stop-line detection')
        pump(3.3)
        check(latest['mission'].phase == 'IDLE', 'cooldown and visual clearance rearm next mission')
        boxes(.05)
        pump(.8)
        check(latest['active'].data and latest['mission'].mission_id == 2, 'next target starts next mission')
        reset_pub.publish(Bool(data=True))
        pump(.1)
        check(not latest['active'].data, 'manual reset requires new confirmation')
        print('RESULT: {} checks passed. Logs: {}'.format(len(passed), directory), flush=True)
    finally:
        for process in reversed(children):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try: process.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        for log in logs: log.close()


if __name__ == '__main__':
    fake_node() if '--fake-yolo-node' in sys.argv else main()
