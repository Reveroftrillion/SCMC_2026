#!/usr/bin/env python3
"""GPS/IMU UDP 실시간 모니터 (제어 명령을 보내지 않음).

python3 sensor_monitor.py
python3 sensor_monitor.py --gps-port 9102 --imu-port 9104
Ctrl+C: 종료. 다른 수신 프로그램을 종료한 뒤 실행하세요.
"""
import argparse
from collections import Counter, deque
import math
import select
import socket
import struct
import sys
import time


def coordinate(value, direction):
    number = float(value)
    degrees = int(number / 100)
    return (degrees + (number - degrees * 100) / 60) * (-1 if direction in ('S', 'W') else 1)


def parse_gps(raw):
    result = []
    for sentence in raw.decode('ascii').strip('\x00\r\n ').splitlines():
        body, checksum = sentence.strip().rsplit('*', 1)
        actual = 0
        for char in body[1:]:
            actual ^= ord(char)
        if not body.startswith('$') or actual != int(checksum[:2], 16):
            raise ValueError('NMEA checksum 오류')
        fields = body.split(',')
        kind = fields[0][-3:]
        if kind == 'RMC':
            valid = fields[2] == 'A'
            detail = '위치 무효'
            if valid:
                lat = coordinate(fields[3], fields[4])
                lon = coordinate(fields[5], fields[6])
                detail = f'위도={lat:.8f} 경도={lon:.8f} 속도={float(fields[7] or 0)*1.852:.2f} km/h'
            result.append((kind, f'UTC={fields[1]} valid={valid} {detail}'))
        elif kind == 'GGA':
            quality = int(fields[6] or 0)
            detail = '위치 무효'
            if quality:
                lat = coordinate(fields[2], fields[3])
                lon = coordinate(fields[4], fields[5])
                detail = f'위도={lat:.8f} 경도={lon:.8f} 고도={fields[9]} m'
            result.append((kind, f'fix={quality} 위성={fields[7]} {detail}'))
        else:
            result.append((kind, sentence.strip()))
    return result


def parse_imu(raw):
    if len(raw) != 115 or raw[:9] != b'#IMUData$' or raw[-2:] != b'\r\n':
        raise ValueError('IMU 패킷 형식 오류')
    if struct.unpack_from('<i', raw, 9)[0] != 88:
        raise ValueError('IMU payload 길이 오류')
    sec, nsec, w, x, y, z, ax, ay, az, lx, ly, lz = struct.unpack_from('<ii10d', raw, 25)
    values = (w, x, y, z, ax, ay, az, lx, ly, lz)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('IMU 유한하지 않은 값')
    norm = math.sqrt(w*w + x*x + y*y + z*z)
    if not 0.9 < norm < 1.1:
        raise ValueError('IMU quaternion norm 오류')
    w, x, y, z = (v / norm for v in (w, x, y, z))
    roll = math.degrees(math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)))
    pitch = math.degrees(math.asin(max(-1, min(1, 2*(w*y-z*x)))))
    yaw = math.degrees(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
    return (f'roll={roll:.2f}° pitch={pitch:.2f}° yaw={yaw:.2f}°\n'
            f'  quaternion(wxyz)=({w:.6f}, {x:.6f}, {y:.6f}, {z:.6f})\n'
            f'  각속도 raw=({ax:.5f}, {ay:.5f}, {az:.5f})\n'
            f'  가속도 raw=({lx:.5f}, {ly:.5f}, {lz:.5f}) timestamp={sec}.{nsec:09d}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bind', default='0.0.0.0')
    parser.add_argument('--gps-port', type=int, default=9102)
    parser.add_argument('--imu-port', type=int, default=9104)
    parser.add_argument('--duration', type=float, default=0, help='자동 종료 초 (0: 계속 실행)')
    args = parser.parse_args()
    sockets = {}
    state = {name: dict(times=deque(), count=0, errors=0, latest=None, source='', data={}, kinds=Counter())
             for name in ('GPS', 'IMU')}
    start = time.monotonic()
    next_draw = start
    try:
        for name, port in (('GPS', args.gps_port), ('IMU', args.imu_port)):
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sockets[sock] = name
            sock.bind((args.bind, port))
            sock.setblocking(False)
        while not args.duration or time.monotonic() - start < args.duration:
            for sock in select.select(list(sockets), [], [], 0.05)[0]:
                raw, source = sock.recvfrom(65535)
                name = sockets[sock]
                item = state[name]
                now = time.monotonic()
                item['count'] += 1
                item['times'].append(now)
                item['source'] = f'{source[0]}:{source[1]}'
                try:
                    records = parse_gps(raw) if name == 'GPS' else [('IMU', parse_imu(raw))]
                    for kind, detail in records:
                        item['data'][kind] = (now, detail)
                        item['kinds'][kind] += 1
                    item['latest'] = now
                except (ValueError, IndexError, UnicodeError, struct.error) as exc:
                    item['errors'] += 1
                    item['data']['오류'] = (now, str(exc))
            now = time.monotonic()
            if now < next_draw:
                continue
            next_draw = now + 0.5
            lines = ['GPS / IMU 실시간 수신 — Ctrl+C 종료 (수신 전용)',
                     'Hz는 최근 3초 UDP 패킷 수 기준. GPS는 GGA/RMC가 각각 한 패킷일 수 있음.']
            for name, item in state.items():
                while item['times'] and item['times'][0] < now - 3:
                    item['times'].popleft()
                hz = len(item['times']) / min(3, max(now-start, 0.001))
                age = now-item['latest'] if item['latest'] is not None else None
                status = '미수신' if age is None else ('정상' if age < 1 else '수신 끊김')
                port = args.gps_port if name == 'GPS' else args.imu_port
                lines.append(f'\n{name} :{port} [{status}] {hz:.1f} packets/s 총={item["count"]} 오류={item["errors"]} 송신={item["source"]}')
                for kind, (stamp, detail) in item['data'].items():
                    lines.append(f'  {kind} ({now-stamp:.1f}초 전, {item["kinds"][kind]}개): {detail}')
            print(('\033[2J\033[H' if sys.stdout.isatty() else '') + '\n'.join(lines), flush=True)
    except KeyboardInterrupt:
        print('\n모니터 종료')
    except OSError as exc:
        print(f'소켓 오류: {exc}\n포트와 IP, 같은 포트를 쓰는 다른 수신 프로그램을 확인하세요.', file=sys.stderr)
        return 1
    finally:
        for sock in sockets:
            sock.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
