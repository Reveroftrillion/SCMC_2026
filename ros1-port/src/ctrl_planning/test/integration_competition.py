"""ctrl_competition.launch.py 를 실제로 띄워 루프백 UDP 가짜 차량으로 한 장면을 돈다 (시뮬 불필요, ~40 s).

    source install/setup.bash
    python3 src/ctrl_planning/test/integration_competition.py

장면: s≈285 에서 출발 → tools/fake_event.py 가 s=329 에 2×3 m 상자를 놓음 → AVOID 로 비켜 지나 s>345.
확인: 차가 움직인다(require_target=true 인데도 behavior_node 가 target 을 낸다), AVOID 진입, |lat_offset| 이
      2 m 이상 나왔다가 0 으로 복귀, FAULT 없음, stop_s 는 한 번도 안 나옴(신호등 없음), Traceback 없음.
가짜 차량 모델은 ctrl_control/test/integration_udp.py 와 같다 (조향 → 요레이트, accel/brake → 가감속).
"""
import csv
import json
import math
import os
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np
import yaml
from pyproj import Transformer

ROOT = Path(__file__).resolve().parents[3]          # .../2026foscar
CONTROL_TEST = ROOT / 'src/ctrl_control/test'
sys.path.insert(0, str(CONTROL_TEST))
from integration_udp import degrees_minutes, nmea   # noqa: E402

START_S, OBSTACLE_S, END_S = 285., 329., 348.


def main():
    with tempfile.TemporaryDirectory(prefix='foscar_comp_test_') as directory:
        directory = Path(directory)
        config = yaml.safe_load((ROOT / 'src/ctrl_bringup/config/low_speed.yaml').read_text())
        config['sensor_bridge']['ros__parameters'].update(sim_ip='127.0.0.1', gps_port=19102, imu_port=19104,
                                                          status_port=19099, ctrl_port=19093)
        config['path_follower']['ros__parameters'].update(sensor_mount_confirmed=True, log_dir=str(directory))
        config_path = directory / 'config.yaml'
        config_path.write_text(yaml.safe_dump(config))
        status = bytearray.fromhex(json.loads((CONTROL_TEST / 'sensor_samples.json').read_text())['9099'])
        inverse = Transformer.from_crs(32652, 4326, always_xy=True)
        points = np.loadtxt(ROOT / 'src/ctrl_bringup/routes/global_path.txt')[:, :2]
        cum = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
        i0 = int(np.searchsorted(cum, START_S))
        x, y = points[i0]
        yaw = math.atan2(points[i0 + 1, 1] - y, points[i0 + 1, 0] - x)
        speed, command = 0., (0., 1., 0.)
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver.bind(('127.0.0.1', 19093))
        receiver.setblocking(False)
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        env = dict(os.environ, ROS_DOMAIN_ID='74', ROS_LOCALHOST_ONLY='1')
        log = (directory / 'launch.log').open('w')
        launch = subprocess.Popen(['ros2', 'launch', 'ctrl_bringup', 'ctrl_competition.launch.py', f'config:={config_path}',
                                   'drive:=true', 'route:=global', 'lidar:=false', 'bag:=false', 'speed_kmh:=40.0'],
                                  env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        echo = subprocess.Popen(['ros2', 'topic', 'echo', '--csv', '/behavior/target', 'ctrl_interfaces/msg/DrivingTarget'], env=env,
                                stdout=(directory / 'target.csv').open('w'), stderr=subprocess.DEVNULL, start_new_session=True)
        fake = None
        start = previous = time.monotonic()
        moved = False
        try:
            while time.monotonic() - start < 90:
                now = time.monotonic()
                elapsed = now - start
                dt = min(0.05, now - previous)
                previous = now
                if launch.poll() is not None:
                    raise AssertionError('launch exited early')
                while True:
                    try:
                        packet, _ = receiver.recvfrom(4096)
                    except BlockingIOError:
                        break
                    body = struct.unpack_from('<bbbfffff', packet, 30)
                    command = body[-3:]
                accel, brake, steer = command
                speed = max(0., speed + (4 * accel - 5 * brake - 0.15) * dt)
                if speed > 0.5 and not moved:
                    moved = True
                    fake = subprocess.Popen([sys.executable, str(ROOT / 'tools/fake_event.py'), 'obstacle', str(OBSTACLE_S)],
                                            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
                angle = math.copysign(float(np.interp(abs(steer), [0, .5, 1], np.radians([0, 15.2, 34.9]))), steer)
                yaw += speed / 3 * math.tan(angle) * dt
                x += speed * math.cos(yaw) * dt
                y += speed * math.sin(yaw) * dt
                sec, nsec = int(elapsed), int((elapsed % 1) * 1e9)
                struct.pack_into('<ii', status, 27, sec, nsec)
                struct.pack_into('<bb', status, 35, 2, 4)
                struct.pack_into('<f', status, 27 + 70, math.degrees(yaw))
                struct.pack_into('<f', status, 27 + 74, speed * 3.6)
                imu_body = struct.pack('<ii10d', sec, nsec, math.cos(yaw / 2), 0., 0., math.sin(yaw / 2), 0., 0., 0., 0., 0., 9.81)
                imu = b'#IMUData$' + struct.pack('<iiii', 88, 0, 0, 0) + imu_body + b'\r\n'
                lon, lat = inverse.transform(x + 302595, y + 4124145)
                gps = nmea(f'GPRMC,{elapsed:010.3f},A,{degrees_minutes(lat)},N,{degrees_minutes(lon)},E,0,0,120926,,,A,')
                for port, packet in [(19099, status), (19104, imu), (19102, gps)]:
                    sender.sendto(packet, ('127.0.0.1', port))
                s_now = cum[int(np.argmin(np.hypot(points[:, 0] - x, points[:, 1] - y)))]
                if moved and s_now > END_S:
                    break
                time.sleep(.02)
            assert moved, 'car never moved (behavior_node silent? require_target holds the car)'
            assert s_now > END_S, f'did not reach s={END_S} (s={s_now:.1f}, v={speed:.1f})'
        finally:
            for proc in (fake, echo, launch):
                if proc is not None and proc.poll() is None:
                    proc.send_signal(signal.SIGINT)
            for proc in (fake, echo, launch):
                if proc is not None:
                    try:
                        proc.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid, signal.SIGKILL)
            log.flush()
            receiver.close()
            sender.close()
        output = (directory / 'launch.log').read_text()
        try:
            check(directory, output)
        except Exception:
            print('--- launch.log tail ---\n' + output[-5000:], flush=True)
            print('--- target.csv head ---\n' + (directory / 'target.csv').read_text()[:1500], flush=True)
            raise


def check(directory, output):
        assert 'Traceback' not in output, output[-4000:]
        rows = list(csv.DictReader(next(directory.glob('follow_*.csv')).open()))
        assert not any(r['state'] == 'FAULT' for r in rows), [r for r in rows if r['state'] == 'FAULT'][:1]
        targets = [line.split(',') for line in (directory / 'target.csv').read_text().splitlines() if line.count(',') >= 6]
        assert targets, 'no /behavior/target seen'
        # --csv 열 순서: header(sec,nanosec,frame_id), state, v_max, stop_s, lat_offset
        states = [t[3] for t in targets]
        offsets = [float(t[6]) for t in targets]
        stops = [t[5] for t in targets]
        assert 'AVOID' in states, f'AVOID never active: {sorted(set(states))}'
        assert max(abs(o) for o in offsets) >= 2.0, f'lat_offset peak {max(offsets, key=abs)}'
        assert abs(offsets[-1]) < 1e-6 and states[-1] == 'CRUISE', f'did not return to CRUISE/0: {states[-1]} {offsets[-1]}'
        assert all(s == 'nan' for s in stops), 'stop_s appeared although no stop_line state is loaded'
        errors = [float(r['cross_track_m']) for r in rows]
        print(f'PASS: {len(targets)} targets, states {sorted(set(states))}, peak offset {max(offsets, key=abs):+.2f} m, '
              f'max cross-track {max(errors):.2f} m, {len(rows)} control rows', flush=True)


if __name__ == '__main__':
    main()
