#!/usr/bin/env python3
"""차량 응답 실측: 제어 명령을 보내면서 Competition Vehicle Status(9099)를 동시에 로깅하고 결과를 계산한다.

  python3 measure.py steer            # steer=1.0 정상선회 → 실효 조향각 (R = v/ω, δ = atan(L/R))
  python3 measure.py steer --steer 0.5
  python3 measure.py accel            # accel 0.3/0.6/1.0 스텝 → vel_x(t), 그리고 brake 1.0 감속
  python3 measure.py log              # 명령 없이 status만 찍어보기
로그: onsite/logs/<test>_<시각>.csv
"""
import argparse, csv, math, os, socket, struct, sys, threading, time
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from go import packet as ctrl_packet

# ---- Competition Vehicle Status (포트 9099, 181B) — 2026-09-12 실측 확정 --------------------
# '#MoraiInfo$' + len(4)=152 + aux(12) + body(152) + '\r\n'
# body = 24.R2 EgoVehicleStatus 에서 타이어 12 float 를 제거한 것. pos/vel_y,z/accel_xyz 는 항상 0.
STATUS_FMT = '<iibbfiff' + 'ffffff' + 'fff' + 'fff' + 'fff' + 'fff' + 'fff' + 'f' + '38s'
STATUS_FIELDS = ('sec nsec ctrl_mode gear signed_vel map_data_id accel brake '
                 'size_x size_y size_z overhang wheelbase rear_overhang '
                 'pos_x pos_y pos_z roll pitch yaw vel_x vel_y vel_z '
                 'ang_vel_x ang_vel_y ang_vel_z accel_x accel_y accel_z steer link_id').split()
assert struct.calcsize(STATUS_FMT) == 152
HDR = 11 + 4 + 12
WHEELBASE = 3.0

def parse_status(raw):
    if len(raw) != HDR + 152 + 2 or raw[:11] != b'#MoraiInfo$':
        return None
    d = dict(zip(STATUS_FIELDS, struct.unpack(STATUS_FMT, raw[HDR:HDR + 152])))
    d['link_id'] = d['link_id'].rstrip(b'\x00').decode('ascii', 'replace')
    return d


class Rig:
    """송신 스레드(50Hz) + 수신 스레드 + CSV 로거."""
    LOG_COLS = ['t', 'cmd_accel', 'cmd_brake', 'cmd_steer', 'ctrl_mode', 'gear', 'accel', 'brake',
                'steer', 'vel_x', 'yaw', 'ang_vel_z', 'link_id']

    def __init__(self, sim_ip, ctrl_port, status_port, logpath):
        self.addr = (sim_ip, ctrl_port)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx.bind(('0.0.0.0', status_port))
        self.rx.settimeout(1.0)
        self.cmd = (0.0, 1.0, 0.0)  # accel, brake, steer — 시작은 브레이크
        self.rows = []
        self.t0 = time.time()
        self.alive = True
        self.status = None
        self.n_rx = 0
        self.f = open(logpath, 'w', newline='')
        self.w = csv.writer(self.f); self.w.writerow(self.LOG_COLS)
        threading.Thread(target=self._tx_loop, daemon=True).start()
        threading.Thread(target=self._rx_loop, daemon=True).start()

    def _tx_loop(self):
        while self.alive:
            a, b, s = self.cmd
            self.tx.sendto(ctrl_packet(a, b, s), self.addr)
            time.sleep(0.02)

    def _rx_loop(self):
        while self.alive:
            try:
                raw, _ = self.rx.recvfrom(65535)
            except socket.timeout:
                continue
            d = parse_status(raw)
            if d is None:
                continue
            self.n_rx += 1
            self.status = d
            row = [round(time.time() - self.t0, 4), *self.cmd, d['ctrl_mode'], d['gear'], d['accel'],
                   d['brake'], d['steer'], d['vel_x'], d['yaw'], d['ang_vel_z'], d['link_id']]
            self.rows.append(row); self.w.writerow(row)

    def set(self, accel=0.0, brake=0.0, steer=0.0):
        self.cmd = (accel, brake, steer)

    def hold(self, sec, label=''):
        end = time.time() + sec
        while time.time() < end:
            s = self.status
            if s:
                print(f'\r  [{label}] t={time.time()-self.t0:6.1f}s  v={s["vel_x"]:6.2f}  '
                      f'yaw={s["yaw"]:8.2f}  wz={s["ang_vel_z"]:8.4f}  mode={s["ctrl_mode"]} gear={s["gear"]}   ',
                      end='')
            time.sleep(0.1)
        print()

    def stop_and_wait(self, max_sec=15):
        self.set(0.0, 1.0, 0.0)
        end = time.time() + max_sec
        while time.time() < end:
            if self.status and abs(self.status['vel_x']) < 0.05:
                break
            time.sleep(0.1)
        time.sleep(0.5)

    def close(self):
        self.stop_and_wait()
        self.alive = False
        time.sleep(0.1)
        self.f.close()


def rows_between(rows, t_from, t_to):
    return [r for r in rows if t_from <= r[0] <= t_to]


def mean(xs):
    return sum(xs) / len(xs) if xs else float('nan')


# ---------------------------------------------------------------------------------------------
def test_steer(rig, steer, accel, hold_sec):
    print(f'\n=== 조향 실측: steer={steer:+.2f}, accel={accel}, {hold_sec}s 유지 ===')
    rig.set(accel, 0.0, 0.0); rig.hold(3, '직진 가속')
    t_on = time.time() - rig.t0
    rig.set(accel, 0.0, steer); rig.hold(hold_sec, '선회')
    t_off = time.time() - rig.t0
    rig.stop_and_wait()

    # 정상상태 = 마지막 40% 구간
    seg = rows_between(rig.rows, t_on + (t_off - t_on) * 0.6, t_off)
    if len(seg) < 10:
        print('데이터 부족'); return
    v_kmh = mean([r[9] for r in seg]); v = v_kmh / 3.6
    wz_raw = mean([r[11] for r in seg])
    # yaw 미분으로 ang_vel_z 단위 확인 (yaw는 deg)
    dyaw = 0.0
    for a, b in zip(seg, seg[1:]):
        d = b[10] - a[10]
        d = (d + 180) % 360 - 180
        dyaw += d
    yaw_rate_deg = dyaw / (seg[-1][0] - seg[0][0])
    print(f'  정상 속도      v = {v_kmh:.2f} km/h = {v:.3f} m/s')
    yaw_rate = math.radians(yaw_rate_deg)                      # rad/s, CCW+
    if abs(abs(wz_raw) - abs(yaw_rate_deg)) < abs(abs(wz_raw) - abs(yaw_rate)):
        unit = 'deg/s'
    else:
        unit = 'rad/s'
    sgn = '같음' if (wz_raw > 0) == (yaw_rate_deg > 0) else '★반대'
    print(f'  status ang_vel_z 평균 = {wz_raw:.4f}   |  yaw 미분 = {yaw_rate_deg:.3f} deg/s = {yaw_rate:.4f} rad/s')
    print(f'  → ang_vel_z 단위: {unit}, yaw 미분과 부호 {sgn}')
    wz = yaw_rate                                              # 이후 계산은 yaw 미분 기준
    if abs(wz) < 1e-4:
        print('  요레이트 0 — 선회 안 함'); return
    R = v / abs(wz)
    delta = math.degrees(math.atan(WHEELBASE / R))
    sign = '+(좌회전, yaw 증가)' if (yaw_rate_deg > 0) == (steer > 0) else '−(steer + 인데 yaw 감소 → 우회전)'
    print(f'  선회반경 R = v/ω = {R:.2f} m')
    print(f'  실효 조향각 δ = atan(L/R) = {delta:.2f}°   (steer 지령 {steer:+.2f} → 풀스케일 환산 {delta/abs(steer):.2f}°)')
    print(f'  부호: steer {steer:+.2f} → yaw rate {yaw_rate_deg:+.2f} deg/s  → {sign}')
    st = mean([r[8] for r in seg])
    print(f'  status.steer 필드 평균 = {st:.3f}  (지령 {steer:+.2f} → 도 단위, 부호 {"같음" if (st>0)==(steer>0) else "★반대"})')
    print(f'  ※ 기구학 자전거모델(L=3.0)로 status.steer {abs(st):.1f}° 라면 R = {WHEELBASE/math.tan(math.radians(abs(st))):.2f} m 여야 함')


def test_accel(rig, levels, hold_sec):
    print(f'\n=== 가속/제동 실측: accel {levels}, 각 {hold_sec}s ===')
    for a in levels:
        rig.stop_and_wait()
        t_on = time.time() - rig.t0
        rig.set(a, 0.0, 0.0); rig.hold(hold_sec, f'accel={a}')
        t_off = time.time() - rig.t0
        rig.set(0.0, 1.0, 0.0); rig.hold(0.5, 'brake')
        t_b0 = t_off
        rig.stop_and_wait()
        t_b1 = time.time() - rig.t0

        seg = rows_between(rig.rows, t_on, t_off)
        if len(seg) < 5:
            print('  데이터 부족'); continue
        v0 = seg[0][9] / 3.6; v1 = seg[-1][9] / 3.6
        # 1초 단위 속도 프로파일
        prof = []
        for k in range(int(hold_sec) + 1):
            rr = rows_between(rig.rows, t_on + k - 0.1, t_on + k + 0.1)
            if rr: prof.append(f'{k}s:{mean([r[9] for r in rr]):.1f}')
        print(f'  accel={a}: vel_x(km/h) ' + ' '.join(prof))
        print(f'           평균 가속도 {(v1 - v0) / (seg[-1][0] - seg[0][0]):.2f} m/s²  (첫 2초: '
              f'{(mean([r[9] for r in rows_between(rig.rows, t_on+1.9, t_on+2.1)])/3.6 - v0)/2:.2f} m/s²)')
        bseg = rows_between(rig.rows, t_b0, t_b1)
        moving = [r for r in bseg if r[9] > 0.5]
        if len(moving) > 2:
            vb0 = moving[0][9] / 3.6; vb1 = moving[-1][9] / 3.6
            dt = moving[-1][0] - moving[0][0]
            print(f'  brake=1.0 from {vb0*3.6:.1f} km/h: {dt:.2f}s 만에 정지, 평균 감속 {(vb0 - vb1)/dt:.2f} m/s²')


def test_log(rig, sec):
    rig.set(0.0, 0.0, 0.0)
    rig.hold(sec, 'log')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('test', choices=['steer', 'accel', 'log'])
    ap.add_argument('--ip', default='192.168.10.1')
    ap.add_argument('--ctrl-port', type=int, default=9093)
    ap.add_argument('--status-port', type=int, default=9099)
    ap.add_argument('--steer', type=float, default=1.0)
    ap.add_argument('--accel', type=float, default=0.2, help='steer 테스트 시 가속 페달')
    ap.add_argument('--hold', type=float, default=None, help='유지 시간(초)')
    ap.add_argument('--levels', default='0.3,0.6,1.0')
    a = ap.parse_args()

    os.makedirs(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs'), exist_ok=True)
    logpath = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs',
                           f'{a.test}_{datetime.now():%m%d_%H%M%S}.csv')
    rig = Rig(a.ip, a.ctrl_port, a.status_port, logpath)
    time.sleep(1.0)
    if rig.status is None:
        print(f'status 수신 없음 (포트 {a.status_port}). 시뮬 연결 확인.'); rig.alive = False; return
    print(f'status 수신 OK ({rig.n_rx} pkt/1s)  link={rig.status["link_id"]}  mode={rig.status["ctrl_mode"]} gear={rig.status["gear"]}')
    try:
        if a.test == 'steer':
            test_steer(rig, a.steer, a.accel, a.hold or 20)
        elif a.test == 'accel':
            test_accel(rig, [float(x) for x in a.levels.split(',')], a.hold or 4)
        else:
            test_log(rig, a.hold or 5)
    except KeyboardInterrupt:
        print('\n중단')
    finally:
        rig.close()
        print(f'로그: {logpath}  ({len(rig.rows)} rows)')


if __name__ == '__main__':
    main()
