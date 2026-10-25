#!/usr/bin/env python3
"""Explicitly authorized short MORAI trial; observe and SIGINT original launch.

No alternative controller, commands, fake sensors, parameter tuning or watchdog
bypass. Passive UDP capture remains alive through the original shutdown handoff.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import socket
import struct
import subprocess
import threading
import time
import xmlrpc.client

import rospy
from ctrl_control.ctrl_core import parse_status, parse_gps, parse_imu
from ctrl_interfaces.msg import VehicleStatus, ControlCommand
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64, String

PORT = Path(__file__).resolve().parents[1]


def safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, dict):
        return {k: safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [safe(v) for v in value]
    return value


def stationary_evidence(samples, shutdown_at, now):
    recent = [v for v in samples if v['t'] >= max(shutdown_at, now-.6)]
    if (len(recent) >= 5 and now-recent[-1]['t'] < .5
            and recent[-1]['t']-recent[0]['t'] >= .4
            and all(abs(v['value']['speed']) < .01 for v in recent)
            and len({v['value']['stamp_hex'] for v in recent}) >= 2):
        return dict(confirmed_at=now, samples=len(recent),
                    duration_s=recent[-1]['t']-recent[0]['t'],
                    maximum_abs_speed_mps=max(abs(v['value']['speed']) for v in recent),
                    last=recent[-1])
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--moving-seconds', type=float, default=3.)
    parser.add_argument('--distance-m', type=float, default=12.)
    parser.add_argument('--wait-seconds', type=float, default=45.)
    args = parser.parse_args()
    # ROS launch nodes default to ROS_HOME cwd; bag paths must be absolute.
    args.output_dir = args.output_dir.resolve()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    master_uri = os.environ.get('ROS_MASTER_URI', 'http://127.0.0.1:11311')
    try:
        xmlrpc.client.ServerProxy(master_uri).getUri('/short_trial')
    except OSError:
        pass
    else:
        raise SystemExit('Existing ROS master: refusing duplicate launch')
    # Check destination ports without owning them during the experiment.
    for port in (909, 1111, 1112, 2368, 9297):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as check:
            check.bind(('127.0.0.1', port))
    started = time.monotonic()
    lock = threading.RLock()
    events = (args.output_dir / 'events.jsonl').open('w', buffering=1)
    counts, parse_errors, state_counts = Counter(), Counter(), Counter()
    last, status_history, controls, telemetry, progress_history, errors = {}, [], [], [], [], []
    interrupted = threading.Event()
    capture_stop = threading.Event()
    child = None
    shutdown_at = None
    reason = 'not_started'
    stop_evidence = None
    sniffer = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0800))
    sniffer.bind(('lo', 0))
    sniffer.settimeout(.1)

    def elapsed():
        return time.monotonic() - started

    def record(kind, value):
        with lock:
            item = dict(t=elapsed(), wall_time=time.time(), kind=kind, value=safe(value))
            events.write(json.dumps(item, allow_nan=False) + '\n')
            counts[kind] += 1
            last[kind] = item
            return item

    def capture():
        while not capture_stop.is_set():
            try:
                frame, address = sniffer.recvfrom(65535)
            except socket.timeout:
                continue
            if address[2] == socket.PACKET_OUTGOING or len(frame) < 42 or frame[23] != 17:
                continue
            offset = 14 + (frame[14] & 15) * 4
            src, dst, length, _ = struct.unpack_from('!HHHH', frame, offset)
            if dst not in (9093, 909, 1111, 1112):
                continue
            data = frame[offset+8:offset+length]
            src_ip, dst_ip = socket.inet_ntoa(frame[26:30]), socket.inet_ntoa(frame[30:34])
            try:
                if dst == 9093:
                    if len(data) != 55 or data[:14] != b'#MoraiCtrlCmd$' or data[-2:] != b'\r\n':
                        raise ValueError('CtrlCmd framing/55-byte length')
                    if struct.unpack_from('<iiii', data, 14) != (23, 0, 0, 0):
                        raise ValueError('CtrlCmd 23-byte payload/reserved fields')
                    mode, gear, command_type, velocity, acceleration, accel, brake, steering = struct.unpack_from('<bbbfffff', data, 30)
                    if not all(math.isfinite(x) for x in (velocity, acceleration, accel, brake, steering)):
                        raise ValueError('Nonfinite control packet')
                    value = dict(source_ip=src_ip, source_port=src, destination_ip=dst_ip, destination_port=dst,
                        mode=mode, gear=gear, command_type=command_type, accel=accel, brake=brake, steering=steering)
                    controls.append(record('udp_control', value))
                elif dst == 909:
                    value = parse_status(data)
                    value.update(source_ip=src_ip, source_port=src, destination_ip=dst_ip,
                                 stamp_hex=data[27:35].hex())
                    status_history.append(record('udp_status', value))
                elif dst == 1111:
                    record('udp_gps', parse_gps(data))
                else:
                    q, angular, linear, stamp = parse_imu(data)
                    record('udp_imu', dict(stamp=stamp))
            except Exception as exc:
                parse_errors[str(dst)] += 1
                record('udp_parse_error', dict(port=dst, bytes=len(data), error=str(exc), header_hex=data[:32].hex()))

    thread = threading.Thread(target=capture, daemon=True)
    thread.start()

    def on_signal(number, frame):
        interrupted.set()

    previous = {number: signal.signal(number, on_signal) for number in (signal.SIGINT, signal.SIGTERM)}

    def callback(topic, msg):
        try:
            if topic == 'status':
                value = dict(speed=msg.speed_mps, mode=msg.ctrl_mode, gear=msg.gear,
                             age_s=time.time()-msg.header.stamp.to_sec())
            elif topic == 's':
                value = msg.data
            elif topic == 'control_state':
                value = msg.data
                state_counts[value.split(' | ', 1)[0]] += 1
            elif topic == 'telemetry':
                value = json.loads(msg.data)
            elif topic == 'request':
                value = dict(state=msg.state, accel=msg.accel, brake=msg.brake, steering_rad=msg.steering_rad,
                             age_s=time.time()-msg.header.stamp.to_sec())
            elif topic == 'odom':
                value = dict(x=msg.pose.pose.position.x, y=msg.pose.pose.position.y,
                             speed=msg.twist.twist.linear.x, age_s=time.time()-msg.header.stamp.to_sec())
            else:
                value = dict(age_s=time.time()-msg.header.stamp.to_sec())
            item = record(topic, value)
            if topic == 'telemetry': telemetry.append(item)
            if topic == 's': progress_history.append(item)
        except Exception as exc:
            errors.append(topic + ': ' + str(exc))

    subscribers = []
    try:
        time.sleep(1.)
        record('launch', dict(drive=True, bag=True, moving_seconds=args.moving_seconds, distance_m=args.distance_m))
        with (args.output_dir / 'launch.log').open('w') as launch_log:
            child_env = dict(os.environ, ROS_LOG_DIR=str(args.output_dir/'ros-logs'),
                             HOME=str(args.output_dir/'home'))
            Path(child_env['HOME']).mkdir(exist_ok=True)
            child = subprocess.Popen(['roslaunch', str(PORT / 'docker/local.launch'), 'drive:=true', 'bag:=true',
                'bag_dir:=' + str(args.output_dir / 'bags')], stdout=launch_log, stderr=subprocess.STDOUT,
                start_new_session=True, env=child_env)
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline:
                if child.poll() is not None or interrupted.is_set():
                    raise RuntimeError('Launch stopped before master startup')
                try:
                    xmlrpc.client.ServerProxy(master_uri).getUri('/short_trial')
                    break
                except OSError:
                    time.sleep(.1)
            else:
                raise RuntimeError('Master startup timeout')
            rospy.init_node('short_trial_observer', disable_signals=True)
            for topic, name, cls in [('status','/vehicle/status',VehicleStatus), ('gps','/sensors/gps',NavSatFix),
                ('imu','/sensors/imu',Imu), ('odom','/localization/odometry',Odometry), ('s','/localization/s',Float64),
                ('control_state','/control/state',String), ('telemetry','/control/telemetry',String),
                ('request','/control/request',ControlCommand)]:
                subscribers.append(rospy.Subscriber(name, cls, lambda msg, key=topic: callback(key, msg), queue_size=100))
            motion_at = None
            first_s = None
            heartbeat = -1.
            reason = 'waiting_for_motion'
            while elapsed() < args.wait_seconds:
                now = elapsed()
                if child.poll() is not None:
                    reason = 'launch_exited'; break
                if interrupted.is_set():
                    reason = 'external_stop'; break
                with lock:
                    status = last.get('udp_status', {})
                    speed = status.get('value', {}).get('speed', 0.)
                    s = last.get('s', {}).get('value')
                    state = last.get('telemetry', {}).get('value', {}).get('state')
                    text = last.get('control_state', {}).get('value', '')
                if first_s is None and s is not None: first_s = s
                if motion_at is None and speed > .2: motion_at = now
                if now - heartbeat >= 1:
                    heartbeat = now
                    update = dict(t=now, speed_mps=speed, s=s, state=state, control_state=text,
                        udp_packets=counts['udp_control'], vehicle=status.get('value'), motion_at=motion_at)
                    (args.output_dir / 'progress.json').write_text(json.dumps(safe(update), indent=2, allow_nan=False)+'\n')
                    print(json.dumps(safe(update), allow_nan=False), flush=True)
                if errors:
                    reason = 'observer_error'; break
                if state in ('FAULT', 'FINISH'):
                    reason = 'controller_' + state; break
                if motion_at is not None:
                    match = re.search(r'error=([-+\d.]+)m', text)
                    if match and abs(float(match.group(1))) > 1.:
                        reason = 'trial_error_stop_over_1m'; break
                    if now - status.get('t', -100) > .3:
                        reason = 'trial_status_stale_stop'; break
                    if now - motion_at >= args.moving_seconds:
                        reason = 'short_motion_time_reached'; break
                    if s is not None and first_s is not None and s-first_s >= args.distance_m:
                        reason = 'short_distance_reached'; break
                    if speed >= 8.:
                        reason = 'trial_speed_stop_8mps'; break
                time.sleep(.02)
            else:
                reason = 'no_motion_timeout'
    except Exception as exc:
        errors.append(str(exc))
        reason = 'exception'
    finally:
        shutdown_at = elapsed()
        record('shutdown_requested', dict(reason=reason, signal='SIGINT'))
        print('STOPPING: ' + reason + '; original bridge braking/Park/Manual handoff', flush=True)
        if child is not None and child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=20)
            except subprocess.TimeoutExpired:
                errors.append('Launch SIGINT timeout')
                os.killpg(child.pid, signal.SIGTERM)
                child.wait(timeout=10)
        # Continue passively observing actual vehicle packets after rospy exits.
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            stop_evidence = stationary_evidence(status_history, shutdown_at, elapsed())
            if stop_evidence:
                break
            time.sleep(.1)
        capture_stop.set()
        thread.join(timeout=1)
        sniffer.close()
        for number, handler in previous.items(): signal.signal(number, handler)
        events.close()
    after = [v for v in status_history if v['t'] >= shutdown_at]
    stopped = bool(stop_evidence)
    auto_d = [v for v in status_history if v['value']['mode'] == 2 and v['value']['gear'] == 4]
    speeds = [v['value']['speed'] for v in status_history]
    progress = [v['value'] for v in progress_history]
    error_values = [float(m.group(1)) for line in (args.output_dir/'events.jsonl').read_text().splitlines()
                    if (entry := json.loads(line))['kind']=='control_state'
                    if (m := re.search(r'error=([-+\d.]+)m', entry['value']))]
    freshness = {key: max((v['value']['age_s'] for line in (args.output_dir/'events.jsonl').read_text().splitlines()
                   if (v := json.loads(line))['kind']==key), default=None) for key in ('gps','imu','status','odom','request')}
    report = dict(scope='short actual MORAI driving trial; no whole-mission/input-extrinsic equivalence claim',
        launch_arguments=['drive:=true','bag:=true'], stop_reason=reason, shutdown_at=shutdown_at,
        launch_returncode=child.returncode if child else None, counts=dict(counts), errors=errors,
        parse_errors=dict(parse_errors), control_states=dict(state_counts),
        udp_control_packets=len(controls), udp_control_first=controls[0] if controls else None,
        udp_control_last=controls[-1] if controls else None,
        auto_d_feedback_count=len(auto_d), auto_d_first=auto_d[0] if auto_d else None,
        maximum_speed_mps=max(speeds, default=0.), initial_status=status_history[0] if status_history else None,
        final_status=after[-1] if after else None, actual_zero_speed_confirmed=stopped,
        stationary_evidence=stop_evidence,
        initial_s=progress[0] if progress else None, final_s=progress[-1] if progress else None,
        delta_s=progress[-1]-progress[0] if progress else None,
        maximum_path_error_m=max(error_values,default=None), maximum_ros_header_age_s=freshness,
        bag_files=[dict(path=str(p),bytes=p.stat().st_size) for p in (args.output_dir/'bags').glob('*')])
    report['short_trial_passed'] = bool(controls and auto_d and max(speeds,default=0.)>.2
        and progress and progress[-1]-progress[0]>.2 and state_counts['CRUISE'] and stopped and not errors)
    (args.output_dir/'report.json').write_text(json.dumps(safe(report),indent=2,allow_nan=False)+'\n')
    print(json.dumps(safe(report),indent=2,allow_nan=False),flush=True)
    if not stopped: print('STOP NOT CONFIRMED: use MORAI Pause immediately.',flush=True)
    return 0 if report['short_trial_passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
