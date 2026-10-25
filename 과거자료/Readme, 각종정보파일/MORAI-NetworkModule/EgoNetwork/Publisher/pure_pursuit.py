"""경로 추종(Pure Pursuit) 계산 로직.

UDP 송수신과 분리되어 있어 시뮬레이터 없이 단위 테스트가 가능하다.
좌표계는 MORAI 월드 좌표(x: 동, y: 북)를 그대로 사용하고,
yaw / steer 모두 양수가 좌회전(반시계) 방향이다.
"""

import math


def find_target_index(path, x, y, lookahead, start_index=0):
    """현재 위치에서 lookahead 거리만큼 앞선 경로점의 인덱스를 돌려준다.

    start_index 이전 구간은 이미 지나온 것으로 보고 무시하므로
    경로가 자기 자신과 가까워지는 구간에서도 뒤로 되돌아가지 않는다.
    lookahead를 넘는 점이 남아 있지 않으면 마지막 점을 목표로 삼는다.
    """
    nearest = start_index
    nearest_dist = math.inf
    for i in range(start_index, len(path)):
        d = math.hypot(path[i][0] - x, path[i][1] - y)
        if d < nearest_dist:
            nearest_dist = d
            nearest = i

    for i in range(nearest, len(path)):
        if math.hypot(path[i][0] - x, path[i][1] - y) >= lookahead:
            return i
    return len(path) - 1


def compute_steer(x, y, yaw, target, wheelbase, max_steer_rad):
    """Pure Pursuit 조향각을 -1 ~ 1 로 정규화해 돌려준다.

    yaw는 radian, target은 월드 좌표의 (x, y).
    """
    dx = target[0] - x
    dy = target[1] - y
    lookahead_dist = math.hypot(dx, dy)
    if lookahead_dist == 0.0:
        return 0.0

    # 월드 좌표의 목표점을 차량 기준 좌표(정면 +x, 좌측 +y)로 회전
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy

    alpha = math.atan2(local_y, local_x)
    delta = math.atan2(2.0 * wheelbase * math.sin(alpha), lookahead_dist)
    return max(-1.0, min(1.0, delta / max_steer_rad))


def distance_to_goal(path, x, y):
    """경로 마지막 점까지의 거리."""
    return math.hypot(path[-1][0] - x, path[-1][1] - y)
