#!/usr/bin/env python3
"""Explicitly authorized MORAI full-course observer; never publishes controls.

Start recorder before production launch. Keep passive wire capture alive through
production bridge shutdown. A new, nonexistent output directory is mandatory.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import runpy
import signal
import shutil
import socket
import struct
import subprocess
import threading
import time
import xmlrpc.client

import numpy as np
import rosgraph
import rospy
import roslib.message
from ctrl_control.ctrl_core import Route, parse_status
from ctrl_control.ctrl_mission_config import read_checkpoints
from ctrl_planning.ctrl_lane_prediction import LaneMap
from course_audit_safety import (DepartureGuard, collision_slots, decode_collision,
                                 input_loss_reason, manual_park_evidence, missing_required_nodes)

ROOT = Path(__file__).resolve().parents[1]
TOPICS = list(dict.fromkeys(runpy.run_path(str(ROOT/'src/ctrl_bringup/scripts/ctrl_record_bag'))['TOPICS'] + [
    '/sensors/gps', '/sensors/imu', '/sensors/lidar/points', '/sensors/camera/traffic/compressed',
    '/perception/traffic_light/debug/compressed', '/control/path', '/tf', '/tf_static', '/rosout_agg']))


def clean(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    return value


def manifest():
    return {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted((ROOT/'src').rglob('*')) if p.is_file()
            and '__pycache__' not in p.parts and p.suffix not in ('.pyc', '.pyo')}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--max-seconds', type=float, default=1500.)
    parser.add_argument('--stall-seconds', type=float, default=120.)
    parser.add_argument('--stop-at-s', type=float, default=1e9)
    parser.add_argument('--launch-file', type=Path, default=ROOT / 'docker/local.launch')
    args = parser.parse_args()
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    (out/'frames').mkdir()
    child_env = dict(os.environ, ROS_HOME=str(out/'ros-home'),
                     ROS_LOG_DIR=str(out/'ros-logs'))
    Path(child_env['ROS_HOME']).mkdir()
    settings = dict(max_seconds=args.max_seconds, stall_seconds=args.stall_seconds, stop_at_s=args.stop_at_s,
                    drive=True, topics=TOPICS, departure_route_m=8.,
                    departure_outside_map_route_m=3.5, departure_duration_s=1.,
                    launch_file=str(args.launch_file.resolve()))
    (out/'settings.json').write_text(json.dumps(settings, indent=2)+'\n')
    frozen = manifest()
    (out/'source-before.json').write_text(json.dumps(frozen, indent=2)+'\n')
    try:
        # Reserve more than the recorded 1.77 GB baseline trial before enabling
        # controls. This guards the observer's storage, not vehicle behavior.
        if shutil.disk_usage(out).free < 10 * 1024**3:
            raise RuntimeError('recording disk reserve below 10 GiB')
        xmlrpc.client.ServerProxy('http://127.0.0.1:11311').getUri('/course_preflight')
    except OSError:
        pass
    else:
        raise RuntimeError('Existing ROS master: refusing duplicate execution')
    for port in (909, 1111, 1112, 2368, 9297):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind(('127.0.0.1', port))
    collision_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    collision_socket.bind(('0.0.0.0', 9092))
    collision_socket.setblocking(False)
    route = Route(np.loadtxt(ROOT/'src/ctrl_bringup/routes/global_path.txt'))
    lane_map = LaneMap.from_file(ROOT/'src/ctrl_bringup/maps/link_set.json')
    checkpoints = read_checkpoints(ROOT/'src/ctrl_bringup/routes/checkpoints.txt')
    checkpoints[0] = dict(xy=np.array([-131.68979755061446, -428.3310229377821]), radius=3.)
    cp = {str(k): dict(min_distance_m=None, closest_s=None, closest_time=None, radius_m=v['radius'],
                      route_s=route.project(*v['xy'])[0]) for k, v in checkpoints.items()}
    started = time.time()
    lock = threading.RLock()
    stopped, capture_stop = threading.Event(), threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopped.set())
    latest, received, subscribers = {}, {}, {}
    counts, wire_counts = Counter(), Counter()
    wire_received = {}
    gaps, transitions, status_samples = [], [], []
    max_s, progress_s, progress_at = -1., -1., started
    previous_target, last_mission = None, None
    image_times, callback_errors = {}, []
    children = []
    launch = recorder = master = None
    reason = 'exception'
    handoff = None
    collision_event = None
    guard = DepartureGuard()
    pcap = (out/'sensors-control.pcap').open('wb')
    pcap.write(struct.pack('<IHHIIII', 0xa1b2c3d4, 2, 4, 0, 0, 65535, 1))
    wire = (out/'wire.jsonl').open('w', buffering=1)
    sniffer = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0800))
    sniffer.bind(('lo', 0))
    sniffer.settimeout(.1)

    def capture():
        nonlocal collision_event
        try:
            while not capture_stop.is_set():
                try:
                    frame, address = sniffer.recvfrom(65535)
                except socket.timeout:
                    continue
                if address[2] == socket.PACKET_OUTGOING or len(frame) < 42 or frame[23] != 17:
                    continue
                offset = 14+(frame[14] & 15)*4
                src, dst, length, _ = struct.unpack_from('!HHHH', frame, offset)
                if dst not in (909, 9093, 1111, 1112, 2368, 9297, 9092):
                    continue
                now = time.time()
                raw = frame[offset+8:offset+length]
                pcap.write(struct.pack('<IIII', int(now), int(now % 1*1e6), len(frame), len(frame)))
                pcap.write(frame)
                with lock:
                    wire_counts[str(dst)] += 1
                    wire_received[str(dst)] = now
                    item = dict(time=now, port=dst)
                    try:
                        if dst == 909:
                            item.update(parse_status(raw), stamp_hex=raw[27:35].hex())
                            status_samples.append(item)
                            del status_samples[:-1000]
                        elif dst == 9093:
                            v = struct.unpack_from('<bbbfffff', raw, 30)
                            item.update(mode=v[0], gear=v[1], accel=v[5], brake=v[6], steering=v[7])
                        elif dst == 9092:
                            item.update(slots=decode_collision(raw), raw_hex=raw.hex())
                            if collision_slots(item['slots']):
                                collision_event = item
                        else:
                            continue
                    except Exception as exc:
                        item['parse_error'] = repr(exc)
                        if dst == 9092:
                            collision_event = item
                    wire.write(json.dumps(clean(item))+'\n')
        except Exception as exc:
            with lock:
                callback_errors.append('capture: '+repr(exc))

    thread = threading.Thread(target=capture, daemon=True)
    thread.start()

    def receive(topic, msg):
        nonlocal previous_target, max_s, progress_s, progress_at, last_mission
        now = time.time()
        with lock:
            counts[topic] += 1
            received[topic] = now
            if topic.endswith('/compressed'):
                if now-image_times.get(topic, 0) >= 2.:
                    image_times[topic] = now
                    name = 'debug' if '/debug/' in topic else 'raw'
                    (out/'frames'/f'{name}-{now:.3f}.jpg').write_bytes(bytes(msg.data))
                return
            item = dict(received=now)
            if hasattr(msg, 'header'):
                item['stamp'] = msg.header.stamp.to_sec()
            if hasattr(msg, 'data') and isinstance(msg.data, (str, float, int)):
                item['data'] = msg.data
                if isinstance(msg.data, str):
                    try:
                        item['parsed'] = json.loads(msg.data)
                    except ValueError:
                        pass
            for field in ('state', 'v_max', 'stop_s', 'lat_offset', 'speed_mps', 'gear',
                          'ctrl_mode', 'accel', 'brake', 'steering_rad', 'entry_s', 'entry_length_m',
                          'reference_id', 'chain_offset', 'chain_start_s', 'chain_length_m'):
                if hasattr(msg, field):
                    item[field] = getattr(msg, field)
            if topic == '/localization/odometry':
                p = msg.pose.pose.position
                item['xy'] = [p.x, p.y]
                q = msg.pose.pose.orientation
                item['yaw'] = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                item['speed'] = msg.twist.twist.linear.x
                projected_s, distance, _ = route.project(p.x, p.y)
                item.update(projected_s=projected_s, route_distance_m=distance)
                for k, v in checkpoints.items():
                    dist = float(np.linalg.norm(np.array([p.x, p.y])-v['xy']))
                    row = cp[str(k)]
                    if row['min_distance_m'] is None or dist < row['min_distance_m']:
                        row.update(min_distance_m=dist, closest_s=projected_s, closest_time=now)
            if topic == '/behavior/markers':
                item['texts'] = [dict(ns=m.ns, text=m.text) for m in msg.markers if m.text]
            if topic == '/localization/s':
                max_s = max(max_s, float(msg.data))
                if msg.data > progress_s+1.:
                    progress_s, progress_at = msg.data, now
            if topic == '/behavior/target':
                if previous_target is not None and now-previous_target > 1.:
                    gaps.append(dict(start=previous_target, end=now, gap_s=now-previous_target))
                previous_target = now
            if topic == '/behavior/mission':
                mission = item.get('parsed', {}).get('mission')
                if mission != last_mission:
                    transitions.append(dict(time=now, s=latest.get('/localization/s', {}).get('data'), mission=mission))
                    last_mission = mission
            # Large arrays remain in the bag, not the 5 Hz observer snapshots.
            if topic not in ('/sensors/lidar/points', '/map/route', '/control/path', '/tf', '/tf_static'):
                latest[topic] = item

    def callback(topic, msg):
        try:
            receive(topic, msg)
        except Exception as exc:
            with lock:
                error = topic+': '+repr(exc)
                if error not in callback_errors:
                    callback_errors.append(error)

    def spawn(command, name):
        log = (out/name).open('w')
        child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                 env=child_env, start_new_session=True)
        children.append((child, log))
        return child

    def stop_child(child, seconds):
        if child and child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=seconds)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                child.wait(timeout=10)

    try:
        master = spawn(['roscore'], 'master.log')
        deadline = time.time()+25.
        while time.time() < deadline:
            try:
                xmlrpc.client.ServerProxy('http://127.0.0.1:11311').getUri('/course_start')
                break
            except OSError:
                time.sleep(.1)
        else:
            raise RuntimeError('master startup timeout')
        rospy.init_node('rmt_fullcourse_observer', disable_signals=True)
        recorder = spawn(['rosbag', 'record', '--buffsize=1024', '--split', '--size=2048',
                          '-O', str(out/'diagnostics')]+TOPICS, 'recorder.log')
        time.sleep(2.)
        if recorder.poll() is not None:
            raise RuntimeError('recorder exited before control launch')
        readiness = None
        deadline = time.time()+10.
        while time.time() < deadline and not stopped.is_set():
            now = time.time()
            with lock:
                parked = manual_park_evidence(status_samples, started, now)
                collision_fresh = now-wire_received.get('9092', 0.) < 1.
                alarm = collision_event or callback_errors
            bags = list(out.glob('diagnostics*.bag.active'))
            if alarm or recorder.poll() is not None:
                raise RuntimeError('pre-control capture or recorder failure')
            if parked and collision_fresh and bags and all(p.stat().st_size > 0 for p in bags):
                readiness = dict(time=now, stationary_manual_park=parked,
                    collision_fresh=True, active_bags=[str(p) for p in bags],
                    disk_free_bytes=shutil.disk_usage(out).free)
                break
            time.sleep(.1)
        if not readiness:
            raise RuntimeError('pre-control recording / fresh Manual-P readiness failed')
        (out/'recording-readiness.json').write_text(json.dumps(readiness, indent=2)+'\n')
        launch = spawn(['roslaunch', str(args.launch_file.resolve()), 'drive:=true', 'bag:=false'], 'launch.log')
        (out/'pids.json').write_text(json.dumps(dict(supervisor=os.getpid(), launch=launch.pid,
            recorder=recorder.pid, master=master.pid))+'\n')
        last_discover = last_report = last_print = 0.
        with (out/'timeline.jsonl').open('w', buffering=1) as timeline:
            while not stopped.is_set():
                now = time.time()
                # Drain CollisionData receiver so packet capture sees continuing traffic.
                for _ in range(128):
                    try:
                        collision_socket.recvfrom(65535)
                    except BlockingIOError:
                        break
                if now-last_discover > 1.:
                    state = rosgraph.Master('/rmt_fullcourse_observer').getSystemState()
                    registered = {name for group in state for _, names in group for name in names}
                    missing = missing_required_nodes(now, started, registered)
                    if missing:
                        reason = 'required_nodes_lost: '+', '.join(missing)
                        break
                    for topic, typename in rospy.get_published_topics():
                        if topic in TOPICS and topic not in subscribers:
                            cls = roslib.message.get_message_class(typename)
                            if cls:
                                subscribers[topic] = rospy.Subscriber(topic, cls,
                                    lambda msg, t=topic: callback(t, msg), queue_size=10)
                    last_discover = now
                with lock:
                    report = clean(dict(start=started, time=now, elapsed_s=now-started,
                        max_s=max_s, no_progress_s=now-progress_at, topics=dict(latest),
                        counts=dict(counts), wire_counts=dict(wire_counts),
                        wire_last_received=dict(wire_received),
                        target_gaps_gt_1s=list(gaps), mission_transitions=list(transitions),
                        checkpoints=cp, callback_errors=list(callback_errors)))
                    odom = latest.get('/localization/odometry', {})
                    speed = latest.get('/vehicle/status', {}).get('speed_mps', 999.)
                    collision = collision_event
                    lost_input = input_loss_reason(now, started, wire_received, received)
                if now-last_report > .2:
                    temp = out/'status.tmp'
                    temp.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
                    temp.replace(out/'status.json')
                    timeline.write(json.dumps(report, ensure_ascii=False)+'\n')
                    last_report = now
                if now-last_print > 20.:
                    print(json.dumps(dict(elapsed_s=now-started, max_s=max_s, speed_mps=speed,
                        state=report['topics'].get('/control/state', {}).get('data'),
                        no_progress_s=now-progress_at)), flush=True)
                    last_print = now
                if collision:
                    reason = 'collision_packet_alarm'
                    (out/'collision-trigger.json').write_text(json.dumps(collision, indent=2)+'\n')
                    break
                if odom and now-odom['received'] < 1.:
                    xy = np.array([odom['xy']])
                    on_road = bool(lane_map.contains_road(xy, seam_tolerance=1.)[0]
                                   or lane_map.contains_intersection_road(xy)[0])
                    if guard.update(now, odom['route_distance_m'], on_road):
                        reason = 'sustained_departure'; break
                if lost_input:
                    reason = lost_input; break
                if callback_errors:
                    reason = 'observer_error'; break
                if launch.poll() is not None or recorder.poll() is not None:
                    reason = 'launch_or_recorder_exited'; break
                if shutil.disk_usage(out).free < 2 * 1024**3:
                    reason = 'recording_disk_reserve_low'; break
                if 'FINISH' in str(latest.get('/control/state', {}).get('data', '')):
                    reason = 'follower_finish'; break
                if max_s >= args.stop_at_s:
                    reason = 'requested_segment_end'; break
                if max_s > route.s[-1]-2. and abs(speed) < .2:
                    reason = 'route_end_stopped'; break
                if now-progress_at > args.stall_seconds:
                    reason = 'no_progress_120s'; break
                if now-started > args.max_seconds:
                    reason = 'time_limit'; break
                stopped.wait(.1)
            else:
                reason = 'operator_stop'
    except Exception as exc:
        reason = 'exception: '+repr(exc)
        print(reason, flush=True)
    finally:
        shutdown_at = time.time()
        print('STOP '+reason, flush=True)
        stop_child(launch, 25.)
        deadline = time.time()+10.
        while time.time() < deadline:
            with lock:
                handoff = manual_park_evidence(status_samples, shutdown_at, time.time())
            if handoff:
                break
            time.sleep(.1)
        stop_child(recorder, 30.)
        rospy.signal_shutdown('course audit ended')
        stop_child(master, 15.)
        capture_stop.set()
        thread.join(timeout=2.)
        sniffer.close()
        collision_socket.close()
        pcap.close()
        wire.close()
        after = manifest()
        (out/'source-after.json').write_text(json.dumps(after, indent=2)+'\n')
        end = clean(dict(reason=reason, max_s=max_s, elapsed_s=time.time()-started,
            shutdown_at=shutdown_at, source_unchanged=frozen == after,
            manual_park_stationary_evidence=handoff, last_raw_status=status_samples[-1:][0] if status_samples else None,
            collision_packets=wire_counts['9092'], collision_monitoring_verified=wire_counts['9092'] > 0,
            callback_errors=callback_errors, checkpoints=cp, mission_transitions=transitions,
            target_gaps_gt_1s=gaps, counts=dict(counts), wire_counts=dict(wire_counts),
            child_exitcodes=[p.poll() for p, _ in children]))
        (out/'end.json').write_text(json.dumps(end, ensure_ascii=False, indent=2)+'\n')
        for _, log in children:
            log.close()
        print(json.dumps(end, ensure_ascii=False), flush=True)
    if not handoff:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
