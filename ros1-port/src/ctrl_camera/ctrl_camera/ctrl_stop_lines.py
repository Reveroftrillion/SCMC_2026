"""정지선 표 — `config/stop_lines.csv` (gps_logger 로 찍은 x, y) 를 **차가 탄 경로**의 s 로 바꾼다.

데이터셋 도구(`ctrl_camera_train` 의 dwell·stoplines)가 쓴다. 주행 중 신호 정지 위치는
`config/yolo_stoplines.json` (yolo_signal_node → behavior_node) 에 따로 있다.

경로 선택은 path_follower 의 `select_route` 와 같은 규칙: 전역 lap 위면 lap (오프셋 0), 아니면
spawn 연결로 (오프셋 = 연결로 길이). 그 결과 s 는 `/localization/s` 와 같은 규약이고, 연결로 위
정지선은 음수 s 로 들어간다. 경로에서 `max_lateral` 이상 떨어진 점은 버린다.
CSV 에 `go` 열이 있으면 그 정지선에서 출발 신호로 볼 색을 `|` 로 나열할 수 있다 (예: `GREEN|LEFT`).
"""

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence

import numpy as np
from ctrl_ros1.ctrl_packages import get_package_share_directory

from ctrl_control.ctrl_core import Route


@dataclass
class Line:
    s: float                                  # 경로 진행거리 [m], /localization/s 와 같은 규약
    go_colors: Sequence[str]                  # 이 정지선에서 출발로 보는 색


def default_file() -> str:
    return str(Path(get_package_share_directory('ctrl_camera')) / 'config/stop_lines.csv')


def default_routes():
    routes = Path(get_package_share_directory('ctrl_bringup')) / 'routes'
    return str(routes / 'global_path.txt'), str(routes / 'spawn_path.txt')


def load_stop_points(path: Path, default_go: Sequence[str]):
    """CSV (gps_logger 형식, x y 필수, go 선택) → [(x, y, go_colors)]."""
    stop_points = []
    with Path(path).open(newline='') as csv_stream:
        for row in csv.DictReader(csv_stream):
            go_colors = (tuple(color.strip().upper() for color in row['go'].split('|') if color.strip())
                  if row.get('go') else tuple(default_go))
            stop_points.append((float(row['x']), float(row['y']), go_colors))
    return stop_points


def project_lines(points, route: Route, s_offset: float, max_lateral: float, logger=None) -> List[Line]:
    """차가 탄 경로에 투영. 경로에서 max_lateral 이상 떨어진 점은 버린다."""
    lines = []
    for line_number, (x, y, go_colors) in enumerate(points, 1):
        route_s, lateral, _ = route.project(x, y)
        if lateral > max_lateral:
            if logger:
                logger.warn(f'정지선 #{line_number} ({x:.1f}, {y:.1f}) 는 이 경로에서 {lateral:.1f} m 떨어져 있어 제외')
            continue
        lines.append(Line(s=float(route_s - s_offset), go_colors=go_colors))
    return sorted(lines, key=lambda ln: ln.s)


def select_route(x, y, global_points, spawn_points, tolerance=5.):
    """path_follower 의 core.select_route 와 같은 규칙 (전역 lap 우선, 아니면 spawn 연결로)."""
    global_route = Route(global_points)
    if global_route.project(x, y)[1] <= tolerance:
        return global_route, 0.
    spawn_route = Route(spawn_points)
    if spawn_route.project(x, y)[1] <= tolerance:
        return spawn_route, float(spawn_route.s[-1] - global_route.s[-1])
    raise ValueError(f'차가 두 경로 모두에서 {min(global_route.project(x, y)[1], spawn_route.project(x, y)[1]):.0f} m 떨어져 있음')


class StopLineTable:
    """route='global'|'spawn' 이면 바로 준비되고, 'auto' 면 첫 pose(`on_pose`) 로 경로를 고른다."""

    def __init__(self, file, path_file, spawn_file, route='auto', max_lateral=1.0,
                 default_go=('GREEN',), logger=None):
        self.logger = logger
        self.max_lateral = float(max_lateral)
        self.points = load_stop_points(file, default_go)
        self.global_points = np.loadtxt(path_file)
        self.spawn_points = np.loadtxt(spawn_file) if spawn_file else self.global_points
        self.lines: Optional[List[Line]] = None
        if route == 'global':
            self.set_route(Route(self.global_points), 0.)
        elif route == 'spawn':
            spawn_route = Route(self.spawn_points)
            self.set_route(spawn_route, float(spawn_route.s[-1] - Route(self.global_points).s[-1]))
        elif route != 'auto':
            raise ValueError(f'route 는 auto|global|spawn: {route!r}')

    @property
    def ready(self) -> bool:
        return self.lines is not None

    def set_route(self, route: Route, s_offset: float) -> None:
        self.lines = project_lines(self.points, route, s_offset, self.max_lateral, self.logger)
        if self.logger:
            self.logger.info(f'경로 {route.s[-1]:.0f} m (s 오프셋 {s_offset:.1f}) 위 정지선 {len(self.lines)}개: '
                             + ', '.join(f'{ln.s:.1f}' for ln in self.lines))

    def on_pose(self, x: float, y: float) -> bool:
        """route='auto' 일 때 첫 pose 로 경로를 고른다. 골랐으면 True."""
        if self.ready:
            return True
        try:
            route, s_offset = select_route(x, y, self.global_points, self.spawn_points)
        except ValueError as e:
            if self.logger:
                self.logger.warn(str(e), throttle_duration_sec=5.0)
            return False
        self.set_route(route, s_offset)
        return True

    def next_line(self, s: float, margin: float = 0.) -> Optional[Line]:
        """s 기준으로 아직 margin 만큼 지나지 않은 첫 정지선. 표가 준비 안 됐으면 None."""
        if not self.ready:
            return None
        return next((ln for ln in self.lines if ln.s + margin > s), None)
