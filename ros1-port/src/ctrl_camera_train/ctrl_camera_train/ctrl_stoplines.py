"""`/localization/s` → 정지선까지의 거리. `ctrl_camera.ctrl_stop_lines.StopLineTable` 을 그대로 쓴다.

`ctrl_extract.py` 의 (조건, 정지선) 단위 분할과 `ctrl_evaluate.py` 의 거리 구간별 혼동행렬이 같은
표를 봐야 한다. 주행 노드들과도 같은 표여야 "평가에선 30 m 인데 주행에선 25 m" 같은
어긋남이 안 생긴다 (명세서 §8 '읽기').

이 패키지는 `ros2 run` 을 안 거치고 venv 에서 직접 돌아가므로 `get_package_share_directory`
가 없을 수 있다. 그래서 share → 소스트리 순으로 파일을 찾고, 둘 다 없으면 **조용히 넘어가지
않고 에러를 낸다** — 정지선 표가 틀리면 거리 평가가 통째로 무의미해진다.
"""

import sys
from pathlib import Path
from typing import List, Optional

SRC = Path(__file__).resolve().parents[2]       # .../src


def _find(pkg: str, rel: str) -> Path:
    try:
        from ctrl_ros1.ctrl_packages import get_package_share_directory
        p = Path(get_package_share_directory(pkg)) / rel
        if p.exists():
            return p
    except Exception:                            # 설치 안 됨 / ament 없음 — 소스트리로 간다
        pass
    p = SRC / pkg / rel
    if p.exists():
        return p
    raise FileNotFoundError(
        f'{pkg}/{rel} 를 찾을 수 없습니다. colcon build 후 source install/setup.bash 하거나 '
        f'--stop-lines-file / --path-file 로 경로를 직접 주세요.')


def _import_table(echo=print):
    """`ctrl_camera.ctrl_stop_lines.StopLineTable`. 워크스페이스가 아직 안 깔렸으면 소스트리로.

    이 패키지는 `ros2 run` 을 안 거치고 venv 에서 직접 돌아간다. `source install/setup.bash`
    를 했으면 그냥 import 되고, 안 했으면 소스트리에서 끌어온다 — 다만 **조용히 하지 않는다.**
    빌드가 오래된 채로 도는 것과 구분이 안 되면 곤란하다.
    """
    try:
        from ctrl_camera.ctrl_stop_lines import StopLineTable
        return StopLineTable
    except ImportError:
        pass
    for pkg in ('ctrl_camera', 'ctrl_control'):
        path = str(SRC / pkg)
        if path not in sys.path:
            sys.path.append(path)
    try:
        from ctrl_camera.ctrl_stop_lines import StopLineTable
    except ImportError as exc:
        raise ImportError(
            f'ctrl_camera.ctrl_stop_lines 를 못 읽었습니다 ({exc}). colcon build 후 '
            f'source install/setup.bash 하거나, ROS 가 PYTHONPATH 에 있는지 확인하세요 '
            f'(/opt/ros/humble/lib/python3.10/site-packages).') from exc
    echo(f'ctrl_camera 를 소스트리에서 읽었습니다 ({SRC}). '
         f'install/ 이 최신인지 확인하세요.')
    return StopLineTable


def load_table(stop_lines_file=None, path_file=None, spawn_file=None,
               route='global', max_lateral=1.0, echo=print):
    """정지선 s 목록을 오름차순으로. route='global' 이면 전역 lap s (= /localization/s 규약)."""
    StopLineTable = _import_table(echo)

    table = StopLineTable(
        file=str(stop_lines_file or _find('ctrl_camera', 'config/stop_lines.csv')),
        path_file=str(path_file or _find('ctrl_bringup', 'routes/global_path.txt')),
        spawn_file=str(spawn_file or _find('ctrl_bringup', 'routes/spawn_path.txt')),
        route=route, max_lateral=max_lateral)
    return table


def stop_line_s(table) -> List[float]:
    return [ln.s for ln in table.lines]


def distance_to_next(s: Optional[float], lines: List[float], lap_length=None) -> Optional[float]:
    """s 에서 다음 정지선까지 남은 거리 [m]. 지난 정지선은 세지 않는다.

    랩을 도는 코스라 마지막 정지선을 지나면 다음 랩 첫 정지선까지가 남는데, 그 구간엔
    볼 신호등이 없으므로 `lap_length` 를 주지 않으면 None 을 돌려준다 (평가에서 제외).
    """
    if s is None or not lines:
        return None
    ahead = [v for v in lines if v >= s]
    if ahead:
        return float(ahead[0] - s)
    if lap_length:
        return float(lines[0] + lap_length - s)
    return None


def nearest_index(s: Optional[float], lines: List[float]) -> Optional[int]:
    """s 에 가장 가까운 정지선의 인덱스. (조건, 정지선) 단위 분할의 그룹 키로 쓴다.

    '다음' 이 아니라 '가장 가까운' 것을 쓴다 — 정지선을 막 지난 프레임과 접근 중인
    프레임은 같은 장면이라 train/val 로 갈라지면 안 된다 (명세서 §5).
    """
    if s is None or not lines:
        return None
    return min(range(len(lines)), key=lambda i: abs(lines[i] - s))
