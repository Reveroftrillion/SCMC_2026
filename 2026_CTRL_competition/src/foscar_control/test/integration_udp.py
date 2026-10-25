"""Real ROS processes, loopback UDP only: finish, command watchdog, GPS loss.

source 2026foscar/install/setup.bash
python3 2026foscar/src/foscar_control/test/integration_udp.py
"""
import csv
import json
import math
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import tempfile
import time

import numpy as np
from pyproj import Transformer
import yaml


ROOT = Path(__file__).resolve().parents[4]


def nmea(body):
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    return f'${body}*{checksum:02X}\r\n'.encode()


def degrees_minutes(degrees):
    integer = int(degrees)
    return f'{integer*100+(degrees-integer)*60:.7f}'


def run_case(case):
    with tempfile.TemporaryDirectory(prefix='foscar_udp_test_') as directory:
        directory = Path(directory)
        config = yaml.safe_load((ROOT/'2026foscar/src/foscar_bringup/config/low_speed.yaml').read_text())
        config['sensor_bridge']['ros__parameters'].update(sim_ip='127.0.0.1', gps_port=19102,
                                                        imu_port=19104, status_port=19099, ctrl_port=19093)
        config['path_follower']['ros__parameters'].update(sensor_mount_confirmed=True, log_dir=str(directory))
        config_path = directory/'config.yaml'
        config_path.write_text(yaml.safe_dump(config))
        captured = json.loads(Path(__file__).with_name('sensor_samples.json').read_text())
        status = bytearray.fromhex(captured['9099'])
        inverse = Transformer.from_crs(32652,4326,always_xy=True)
        points = np.loadtxt(ROOT/'2026foscar/src/foscar_bringup/routes/global_path.txt')
        x,y = points[0,:2]
        yaw = math.atan2(points[1,1]-y,points[1,0]-x)
        speed = 0.
        command = (0.,1.,0.)
        receiver = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        receiver.bind(('127.0.0.1',19093));receiver.setblocking(False)
        sender = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        environment = dict(os.environ, ROS_DOMAIN_ID='73', ROS_LOCALHOST_ONLY='1')
        log = (directory/'launch.log').open('w')
        process = subprocess.Popen(['ros2','launch','foscar_bringup','low_speed.launch.py',
                                    f'config:={config_path}','drive:=true','distance_m:=5.0','route:=global'],
                                   env=environment,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        start = previous = time.monotonic()
        first_accel = interruption = follower_pid = None
        braking_after_fault = False
        command_times = []
        try:
            while time.monotonic()-start < 18:
                now = time.monotonic();elapsed = now-start
                dt = min(0.05, now-previous);previous = now
                if process.poll() is not None:
                    raise AssertionError('launch exited early')
                if first_accel is not None and interruption is None and now-first_accel > 1.5 and case != 'finish':
                    interruption = now
                    if case == 'command_loss':
                        import re
                        matches = re.findall(r'\[path_follower-3\]: process started with pid \[(\d+)\]', (directory/'launch.log').read_text())
                        follower_pid = int(matches[-1])
                        os.kill(follower_pid,signal.SIGSTOP)
                while True:
                    try:
                        packet,_ = receiver.recvfrom(4096)
                    except BlockingIOError:
                        break
                    assert len(packet) == 55
                    command_times.append(now)
                    body = struct.unpack_from('<bbbfffff',packet,30)
                    assert body[:3] == (2,4,1)
                    command = body[-3:]
                    if command[0] > 0 and first_accel is None:
                        first_accel = now
                    if interruption is not None and now-interruption > 0.35 and command[0] == 0 and command[1] == 1:
                        braking_after_fault = True
                accel, brake, steer = command
                speed = max(0.,speed+(4*accel-5*brake-0.15)*dt)
                angle = math.copysign(float(np.interp(abs(steer),[0,.5,1],np.radians([0,15.2,34.9]))),steer)
                yaw += speed/3*math.tan(angle)*dt
                x += speed*math.cos(yaw)*dt;y += speed*math.sin(yaw)*dt
                sec,nsec = int(elapsed),int((elapsed%1)*1e9)
                struct.pack_into('<ii',status,27,sec,nsec)
                struct.pack_into('<bb',status,35,2,4)
                struct.pack_into('<f',status,27+70,math.degrees(yaw))
                struct.pack_into('<f',status,27+74,speed*3.6)
                # Offsets verified against Competition status parser below.
                from foscar_control.core import parse_status
                parsed = parse_status(status)
                assert abs(parsed['yaw']-yaw) < 1e-5 and abs(parsed['speed']-speed) < 1e-5
                imu_body = struct.pack('<ii10d',sec,nsec,math.cos(yaw/2),0.,0.,math.sin(yaw/2),0.,0.,0.,0.,0.,9.81)
                imu = b'#IMUData$'+struct.pack('<iiii',88,0,0,0)+imu_body+b'\r\n'
                lon,lat = inverse.transform(x+302595,y+4124145)
                valid = not (case == 'gps_loss' and interruption is not None)
                gps = nmea(f'GPRMC,{elapsed:010.3f},{"A" if valid else "V"},{degrees_minutes(lat)},N,{degrees_minutes(lon)},E,0,0,120926,,,A,')
                for port,packet in [(19099,status),(19104,imu),(19102,gps)]:
                    sender.sendto(packet,('127.0.0.1',port))
                if case != 'finish' and interruption is not None and now-interruption > 1.5:
                    break
                if case == 'finish' and 'FINISH |' in (directory/'launch.log').read_text() and speed < .01:
                    break
                time.sleep(.02)
            assert first_accel is not None,'no acceleration command'
            if case != 'finish':
                assert braking_after_fault,'watchdog did not command full brake'
            if follower_pid is not None:
                os.kill(follower_pid,signal.SIGCONT);follower_pid = None
            process.send_signal(signal.SIGINT)
            process.wait(timeout=8)
            log.flush()
            output = (directory/'launch.log').read_text()
            assert 'Traceback' not in output,output[-4000:]
            rows = list(csv.DictReader(next(directory.glob('follow_*.csv')).open()))
            if case == 'finish':
                assert any(r['state']=='FINISH' for r in rows),output[-4000:]
                assert max(float(r['cross_track_m']) for r in rows)<0.5
            if case == 'gps_loss':
                assert any(r['state']=='FAULT' for r in rows),output[-4000:]
            print(f'PASS {case}: {len(command_times)} commands, final speed={speed:.3f}m/s',flush=True)
        except Exception:
            log.flush()
            print((directory/'launch.log').read_text()[-5000:],flush=True)
            raise
        finally:
            if follower_pid is not None:
                os.kill(follower_pid,signal.SIGCONT)
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL);process.wait()
            receiver.close();sender.close();log.close()


if __name__ == '__main__':
    for case in ('finish','command_loss','gps_loss'):
        run_case(case)
