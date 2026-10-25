"""기록된 경로를 Pure Pursuit 으로 주행.

9099 로 Ego 상태를 받고, 9093 으로 EgoCtrlCmd(velocity + steer)를 보낸다.
시뮬레이터에서 'Q' 를 눌러 Ego Controller 를 AV - ExternalCtrl 로 바꿔야 동작한다.
"""

import csv
import math
import sys
import time
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[2]))

from lib.network.UDP import Receiver, Sender
from lib.define.EgoVehicleStatus import EgoVehicleStatus
from lib.define.EgoCtrlCmd import EgoCtrlCmd
from pure_pursuit import find_target_index, compute_steer, distance_to_goal

RECV_IP, RECV_PORT = '0.0.0.0', 9099
SEND_IP, SEND_PORT = '127.0.0.1', 9093

PATH_FILE = Path(__file__).resolve().parent / 'path.csv'

# --- 튜닝 값 -----------------------------------------------------------------
V_MAX = 30.0            # km/h, 직선 목표 속도
V_MIN = 10.0            # km/h, 급커브 목표 속도
MAX_STEER_RAD = 0.6     # rad, 차량 최대 조향각 (약 35도) - 차종에 따라 조절
WHEELBASE = 3.0         # m, 상태 패킷에 값이 있으면 그걸 우선 사용
LD_GAIN = 0.4           # lookahead = LD_GAIN * 속도(m/s) + LD_MIN
LD_MIN, LD_MAX = 3.0, 12.0
GOAL_TOL = 2.0          # m, 경로 끝까지 이 거리 안에 들어오면 정지
PERIOD = 0.05           # s, 20Hz 제어 주기
YAW_IS_DEGREE = True    # MORAI yaw 단위. 차가 엉뚱하게 돌면 False 로 바꿔볼 것
# -----------------------------------------------------------------------------


def load_path(path_file):
    with open(path_file, newline='') as f:
        reader = csv.DictReader(f)
        points = [(float(row['x']), float(row['y'])) for row in reader]
    if len(points) < 2:
        raise SystemExit(f'{path_file} 에 경로점이 부족합니다. record_path.py 를 먼저 실행하세요.')
    return points


def make_cmd():
    cmd = EgoCtrlCmd()
    cmd.ctrl_mode = 2   # AutoMode
    cmd.gear = 4        # D
    cmd.cmd_type = 2    # Velocity(velocity, steer)
    return cmd


def target_velocity(steer):
    """조향이 클수록 감속."""
    return V_MIN + (V_MAX - V_MIN) * (1.0 - abs(steer))


def main():
    path = load_path(PATH_FILE)
    print(f'경로 {len(path)}점 로드: {PATH_FILE}')

    receiver = Receiver(RECV_IP, RECV_PORT, EgoVehicleStatus())
    sender = Sender(SEND_IP, SEND_PORT)
    cmd = make_cmd()

    last_index = 0
    try:
        while True:
            time.sleep(PERIOD)
            status = receiver.get_data()
            x, y, yaw_raw = status.pos_x, status.pos_y, status.yaw
            if x == 0.0 and y == 0.0:
                continue  # 아직 첫 패킷이 안 들어옴

            yaw = math.radians(yaw_raw) if YAW_IS_DEGREE else yaw_raw
            wheelbase = status.wheelbase if status.wheelbase > 0 else WHEELBASE
            speed_ms = abs(status.signed_vel)

            if distance_to_goal(path, x, y) < GOAL_TOL:
                break

            lookahead = min(LD_MAX, max(LD_MIN, LD_GAIN * speed_ms + LD_MIN))
            last_index = find_target_index(path, x, y, lookahead, last_index)
            target = path[last_index]

            cmd.steer = compute_steer(x, y, yaw, target, wheelbase, MAX_STEER_RAD)
            cmd.velocity = target_velocity(cmd.steer)
            sender.send(cmd)
    except KeyboardInterrupt:
        print('\n사용자 중단')

    print('\n정지 명령 전송')
    cmd.steer = 0.0
    cmd.velocity = 0.0
    for _ in range(10):
        sender.send(cmd)
        time.sleep(PERIOD)
    print('완료')


if __name__ == '__main__':
    main()
