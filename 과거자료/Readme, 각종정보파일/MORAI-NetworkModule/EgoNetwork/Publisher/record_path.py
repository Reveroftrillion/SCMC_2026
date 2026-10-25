"""주행 경로 기록.

시뮬레이터에서 수동(또는 아무 방식으로든) 주행하는 동안 Ego 위치를 받아
path.csv 로 저장한다. Ctrl+C 로 종료한다.
"""

import csv
import math
import sys
import time
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[2]))

from lib.network.UDP import Receiver
from lib.define.EgoVehicleStatus import EgoVehicleStatus

IP = '0.0.0.0'
PORT = 9099

OUTPUT = Path(__file__).resolve().parent / 'path.csv'
MIN_DIST = 0.3      # m, 직전 저장점과 이 거리 이상 떨어져야 기록 (중복 제거)
PERIOD = 0.05       # s, 20Hz 폴링


def main():
    receiver = Receiver(IP, PORT, EgoVehicleStatus())
    print(f'{IP}:{PORT} 수신 대기 중... (Ctrl+C 로 종료)')

    last = None
    count = 0
    with open(OUTPUT, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['x', 'y', 'z', 'yaw'])
        try:
            while True:
                time.sleep(PERIOD)
                status = receiver.get_data()
                x, y, z, yaw = status.pos_x, status.pos_y, status.pos_z, status.yaw

                if x == 0.0 and y == 0.0:
                    continue  # 아직 첫 패킷이 안 들어옴
                if last is not None and math.hypot(x - last[0], y - last[1]) < MIN_DIST:
                    continue

                writer.writerow([x, y, z, yaw])
                f.flush()
                last = (x, y)
                count += 1
                print(f'\r기록 {count}점  x={x:.2f} y={y:.2f} yaw={yaw:.1f}', end='')
        except KeyboardInterrupt:
            pass

    print(f'\n{count}개 점을 {OUTPUT} 에 저장했습니다.')


if __name__ == '__main__':
    main()
