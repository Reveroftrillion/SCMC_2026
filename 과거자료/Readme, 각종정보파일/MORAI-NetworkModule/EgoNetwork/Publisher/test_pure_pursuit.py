import math
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))

from pure_pursuit import find_target_index, compute_steer, distance_to_goal

STRAIGHT = [(float(i), 0.0) for i in range(11)]


def test_target_is_first_point_beyond_lookahead():
    assert find_target_index(STRAIGHT, 0.0, 0.0, lookahead=3.0) == 3


def test_target_never_goes_backwards_from_start_index():
    # 차량이 (2,0)으로 되돌아와 있어도 이미 8번 점까지 진행했다면 뒤로 가지 않는다
    assert find_target_index(STRAIGHT, 2.0, 0.0, lookahead=1.0, start_index=8) == 8


def test_target_clamps_to_last_point_near_end_of_path():
    assert find_target_index(STRAIGHT, 9.5, 0.0, lookahead=5.0) == 10


def test_straight_ahead_target_gives_zero_steer():
    steer = compute_steer(0.0, 0.0, 0.0, (5.0, 0.0), wheelbase=3.0, max_steer_rad=0.6)
    assert steer == 0.0


def test_target_on_the_left_gives_positive_steer():
    steer = compute_steer(0.0, 0.0, 0.0, (5.0, 5.0), wheelbase=3.0, max_steer_rad=0.6)
    assert steer > 0.0


def test_target_on_the_right_gives_negative_steer():
    steer = compute_steer(0.0, 0.0, 0.0, (5.0, -5.0), wheelbase=3.0, max_steer_rad=0.6)
    assert steer < 0.0


def test_steer_is_clamped_to_unit_range():
    steer = compute_steer(0.0, 0.0, 0.0, (0.1, 5.0), wheelbase=3.0, max_steer_rad=0.6)
    assert steer == 1.0


def test_vehicle_heading_is_accounted_for():
    # 차량이 90도(좌) 방향을 보고 있고 목표가 정북(0,5)이면 정면이므로 조향 0
    steer = compute_steer(0.0, 0.0, math.pi / 2, (0.0, 5.0), wheelbase=3.0, max_steer_rad=0.6)
    assert abs(steer) < 1e-9


def test_distance_to_goal_measures_last_path_point():
    assert distance_to_goal(STRAIGHT, 7.0, 0.0) == 3.0
