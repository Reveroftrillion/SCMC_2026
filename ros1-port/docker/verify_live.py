#!/usr/bin/env python3
"""Launch drive:=false, observe real ROS/TF, and passively count control UDP.

Requires CAP_NET_RAW for capture and NET_BIND_SERVICE for status port 909.
Never publishes sensor inputs, changes clock, or enables control transmission.
"""
import argparse
from collections import Counter
import json
import math
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import threading
import time
import xmlrpc.client

import rosgraph
import roslib.message
import rospy
import tf2_ros

PORT = Path(__file__).resolve().parents[1]
TOPICS = ['/sensors/gps', '/sensors/imu', '/vehicle/status', '/sensors/lidar/points',
          '/sensors/camera/traffic/compressed', '/localization/odometry', '/localization/s',
          '/localization/mode', '/map/route', '/tf', '/tf_static', '/behavior/target',
          '/behavior/state', '/behavior/mission', '/control/request', '/control/state',
          '/perception/traffic_signal', '/events', '/perception/footprints']


def json_value(value):
    if isinstance(value, float) and not math.isfinite(value):
        return 'NaN' if math.isnan(value) else str(value)
    if isinstance(value, dict):
        return {key: json_value(item) for key, item in value.items()}
    if isinstance(value, list):
        return [json_value(item) for item in value]
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=40)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--launch-file', type=Path, default=PORT / 'docker/local.launch')
    parser.add_argument('--kill-node', help='test required-node shutdown using rosnode kill')
    parser.add_argument('--kill-after', type=float, default=8.)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    master_uri = os.environ.get('ROS_MASTER_URI', 'http://127.0.0.1:11311')
    try:
        xmlrpc.client.ServerProxy(master_uri).getUri('/rmt_verification')
    except OSError:
        pass
    else:
        raise SystemExit('A ROS master already exists; stop it or use a separate ROS_MASTER_URI')
    sniffer = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0800))
    sniffer.bind(('lo', 0))
    sniffer.settimeout(.2)
    control_packets = []
    stopping = threading.Event()

    def capture():
        while not stopping.is_set():
            try:
                frame = sniffer.recv(65535)
            except socket.timeout:
                continue
            if len(frame) < 42 or frame[23] != 17:
                continue
            ihl = (frame[14] & 15) * 4
            start = 14 + ihl
            src, dst = struct.unpack_from('!HH', frame, start)
            if dst == 9093:
                control_packets.append(dict(source_port=src, destination_port=dst,
                                            payload_prefix_hex=frame[start+8:start+24].hex()))

    thread = threading.Thread(target=capture, daemon=True)
    thread.start()
    launch_log = args.output.with_suffix('.launch.log')
    counts, last, times = Counter(), {}, {}
    subscribers = {}
    report = dict(scope='real MORAI UDP and ROS1 TCPROS, drive=false; current camera differs from source conditions',
                  topics={}, errors=[], control_udp_destination_9093=control_packets)

    def receive(topic, cls, raw):
        message = raw
        counts[topic] += 1
        now = time.monotonic()
        times.setdefault(topic, [now, now])[1] = now
        item = dict(type=message._type)
        if hasattr(message, 'header'):
            item.update(frame_id=message.header.frame_id, stamp_ns=message.header.stamp.to_nsec())
        for field in ('state', 'speed_mps', 'gear', 'v_max', 'stop_s', 'accel', 'brake', 'steering_rad', 'width', 'height'):
            if hasattr(message, field):
                item[field] = getattr(message, field)
        if message._type in ('std_msgs/String', 'std_msgs/Float64'):
            item['data'] = message.data[:1500] if isinstance(message.data, str) else message.data
        if message._type == 'sensor_msgs/CompressedImage':
            import cv2
            import numpy as np
            image = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR)
            item['decoded_shape'] = list(image.shape) if image is not None else None
        last[topic] = item

    def callback(topic, cls, message):
        try:
            receive(topic, cls, message)
        except Exception as exc:
            error = topic + ': ' + str(exc)
            if error not in report['errors']:
                report['errors'].append(error)

    with launch_log.open('w') as log:
        child = subprocess.Popen(['roslaunch', str(args.launch_file.resolve()), 'drive:=false'],
                                 stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline:
                if child.poll() is not None:
                    raise RuntimeError('roslaunch exited before master was available')
                try:
                    xmlrpc.client.ServerProxy(master_uri).getUri('/rmt_verification')
                    break
                except OSError:
                    time.sleep(.2)
            else:
                raise RuntimeError('ROS master startup timeout')
            rospy.init_node('rmt_readonly_verification', disable_signals=True)
            tf_buffer = tf2_ros.Buffer()
            tf_listener = tf2_ros.TransformListener(tf_buffer)
            deadline = time.monotonic() + args.seconds
            observe_started = time.monotonic()
            killed = False
            while time.monotonic() < deadline and child.poll() is None:
                if rospy.has_param('/sensor_bridge/drive'):
                    report['drive_parameter'] = rospy.get_param('/sensor_bridge/drive')
                    if report['drive_parameter'] is not False:
                        raise RuntimeError('Expected drive=false; terminating launch')
                for topic, name in rospy.get_published_topics():
                    if topic in TOPICS and topic not in subscribers:
                        cls = roslib.message.get_message_class(name)
                        if cls:
                            subscribers[topic] = rospy.Subscriber(topic, cls,
                                lambda message, t=topic, c=cls: callback(t, c, message), queue_size=10)
                if args.kill_node and not killed and time.monotonic() - observe_started >= args.kill_after:
                    subprocess.run(['rosnode', 'kill', args.kill_node], check=True, stdout=log, stderr=log)
                    report['killed_node'] = args.kill_node
                    killed = True
                    child.wait(timeout=15)
                    break
                time.sleep(.2)
            report['transport_parameters'] = {key: rospy.get_param('/sensor_bridge/' + key, None)
                for key in ('sim_ip', 'bind_ip', 'gps_port', 'imu_port', 'status_port', 'ctrl_port')}
            try:
                report['node_names'] = rosgraph.Master('/rmt_verification').getSystemState()
            except OSError:
                if not args.kill_node:
                    raise
            report['tf'] = {}
            for target, source in [('map', 'base_link'), ('base_link', 'lidar')]:
                try:
                    tf = tf_buffer.lookup_transform(target, source, rospy.Time(0))
                    report['tf'][target + '<-' + source] = dict(
                        translation=[getattr(tf.transform.translation, a) for a in ('x', 'y', 'z')],
                        rotation=[getattr(tf.transform.rotation, a) for a in ('x', 'y', 'z', 'w')])
                except Exception as exc:
                    report['tf'][target + '<-' + source] = dict(error=str(exc))
        except Exception as exc:
            if args.kill_node and report.get('killed_node') and isinstance(exc, OSError):
                report['expected_post_shutdown_master_disconnect'] = str(exc)
            else:
                report['errors'].append(str(exc))
        finally:
            report['launch_returncode_before_shutdown'] = child.poll()
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGTERM)
                    child.wait(timeout=10)
                    report['errors'].append('roslaunch required SIGTERM after SIGINT timeout')
            report['launch_returncode'] = child.returncode
            stopping.set()
            thread.join(timeout=1)
            sniffer.close()
    for topic in TOPICS:
        span = times[topic][1] - times[topic][0] if topic in times else 0
        report['topics'][topic] = dict(count=counts[topic],
            observed_hz=(counts[topic] - 1) / span if span else None, last=last.get(topic))
    required = ['/sensors/gps', '/sensors/imu', '/vehicle/status', '/sensors/lidar/points',
                '/sensors/camera/traffic/compressed', '/localization/odometry',
                '/behavior/target', '/control/request']
    report['passed_receive_and_decisions'] = (not report['errors']
        and report.get('drive_parameter') is False and not control_packets
        and all(counts[t] for t in required)
        and all('error' not in tf for tf in report.get('tf', {}).values())
        and len(report.get('tf', {})) == 2)
    report['source_sensor_equivalence'] = ('NOT VERIFIED: Camera-7 resolution/pitch differ; original camera position/FOV '
        'and MORAI vehicle-origin to base_link/rear-axle transform unknown; actual LiDAR/GPS mounts differ numerically')
    if args.kill_node:
        report['passed_required_node_shutdown'] = (report.get('killed_node') == args.kill_node
            and report['launch_returncode_before_shutdown'] is not None and not control_packets
            and not report['errors'] and report.get('drive_parameter') is False)
        # Shutdown can close the master before post-shutdown parameter/TF reads.
        report['passed_receive_and_decisions'] = False
    args.output.write_text(json.dumps(json_value(report), indent=2, allow_nan=False) + '\n')
    print(json.dumps(json_value(report), indent=2, allow_nan=False))
    if not report.get('passed_required_node_shutdown', report['passed_receive_and_decisions']):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
