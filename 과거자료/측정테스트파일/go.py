#!/usr/bin/env python3
"""노트북 → 시뮬레이터(192.168.10.1:9093) 로 제어 명령을 50Hz로 쏴서 차를 움직인다.

사용법:
    python3 go.py                  # 액셀 0.3 으로 직진
    python3 go.py --accel 0.5 --steer 0.3   # 좌로 꺾으며 전진 (steer: -1~1, 좌+)
    python3 go.py --brake 1.0      # 정지
Ctrl+C 로 종료하면 자동으로 브레이크 패킷을 보내고 멈춘다.

패킷 규격: '#MoraiCtrlCmd$' + len(4B) + aux(12B) + payload 23B + '\\r\\n' = 55B
  payload = <bbbfffff : ctrl_mode(2) gear(4) cmd_type(1) velocity(0) acceleration(0) accel brake steer
"""
import argparse, socket, struct, time, sys

TAIL = b'\r\n'

def packet(accel, brake, steer, short=True):
    # mode=2(AutoMode) gear=4(D) cmd_type=1(Throttle) velocity=0 acceleration=0
    # ★ 대회 빌드(S4.MolitComp03)는 23B payload (rear_steer 없음) — 2026-09-12 실측 확인.
    #   24.R2 공식 예제의 27B(rear_steer 포함)는 이 빌드에서 무시됨.
    if short:
        body = struct.pack('<bbbfffff', 2, 4, 1, 0.0, 0.0, accel, brake, steer)
    else:
        body = struct.pack('<bbbffffff', 2, 4, 1, 0.0, 0.0, accel, brake, steer, 0.0)
    header = b'#MoraiCtrlCmd$' + struct.pack('<iiii', len(body), 0, 0, 0)
    return header + body + TAIL

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ip', default='192.168.10.1')
    ap.add_argument('--port', type=int, default=9093)
    ap.add_argument('--accel', type=float, default=0.3)
    ap.add_argument('--brake', type=float, default=0.0)
    ap.add_argument('--steer', type=float, default=0.0)
    ap.add_argument('--hz', type=float, default=50)
    ap.add_argument('--long', action='store_true', help='24.R2 규격 27바이트(rear_steer 포함)로 전송. 대회 빌드에선 안 먹힘')
    a = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    pkt = packet(a.accel, a.brake, max(-1.0, min(1.0, a.steer)), not a.long)
    print(f'→ {a.ip}:{a.port}  [{len(pkt)}B]  accel={a.accel} brake={a.brake} steer={a.steer}  ({a.hz:.0f}Hz, Ctrl+C 로 정지)')
    n = 0
    try:
        while True:
            sock.sendto(pkt, (a.ip, a.port))
            n += 1
            if n % int(a.hz) == 0:
                print(f'  {n} packets sent', end='\r'); sys.stdout.flush()
            time.sleep(1.0 / a.hz)
    except KeyboardInterrupt:
        stop = packet(0.0, 1.0, 0.0, not a.long)
        for _ in range(50):
            sock.sendto(stop, (a.ip, a.port)); time.sleep(0.02)
        print(f'\n정지 명령 전송 완료 (총 {n} packets)')

if __name__ == '__main__':
    main()
