"""이번 지역변수 정리를 고정된 작업 전 커밋과 AST로 비교한다.

이름을 역매핑한 AST 전체가 같아야 한다. 기존 바이트 보존 검사와 차등
테스트를 대체하지 않으며 기존 결과 JSON/manifest를 기록하지 않는다.
"""
import ast
import copy
from pathlib import Path
import subprocess
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from ctrl_names import MAP, legacy_ast


REPO = Path(__file__).resolve().parents[2]
BASE = 'c185c84024824fb5c2f4136e91a6b7b4e7bd02e7'
# 공개 인자/속성/함수명은 포함하지 않는다. 키는 클래스와 함수의 lexical 경로다.
RENAMES = {
    'ctrl_camera/ctrl_yolo_onnx.py': {
        'decode': {'classes': 'class_ids', 'keep': 'valid_rows', 'out': 'detections',
                   'i': 'detection_index', 'x': 'box_x', 'y': 'box_y',
                   'w': 'box_width', 'h': 'box_height', 'xy': 'image_box'},
        'choose_signal': {'h': 'image_height', 'w': 'image_width', 'd': 'detection',
                          'x': 'normalized_center_x', 'y': 'normalized_center_y'},
        'Detector.__init__': {'inp': 'model_input', 'p_': 'provider_name', 'i': 'class_index'},
        'Detector.__call__': {'h': 'image_height', 'w': 'image_width',
                              'nw': 'resized_width', 'nh': 'resized_height'},
    },
    'ctrl_camera/ctrl_stop_lines.py': {
        'load_stop_points': {'out': 'stop_points', 'f': 'csv_stream',
                             'go': 'go_colors', 'c': 'color'},
        'project_lines': {'i': 'line_number', 'go': 'go_colors', 's': 'route_s'},
        'select_route': {'lap': 'global_route', 'spawn': 'spawn_route'},
        'StopLineTable.__init__': {'spawn': 'spawn_route'},
        'StopLineTable.on_pose': {'offset': 's_offset'},
    },
    'ctrl_control/ctrl_reference.py': {
        'ReferenceLine.__init__': {'gs': 'global_stations', 'pts': 'reference_points'},
        'ReferenceLine.valid_for': {'q': 'local_station'},
    },
    'ctrl_control/ctrl_reference_control.py': {
        'step_reference': {'old': 'previous_reference', 'global_local': 'route_station',
                           'global_error': 'global_lateral_error', 'i': 'profile_index',
                           'j': 'next_profile_index', 'look_max': 'max_lookahead',
                           'inner': 'reference_controller', 'stations': 'route_stations',
                           'same': 'matching_suffix', 'original_cap': 'original_speed_cap',
                           'decel': 'deceleration', 'source': 'state_source', 'out': 'output'},
    },
    'ctrl_control/ctrl_speed_policy.py': {
        'uncapped_highway_zones': {'begin': 'start_s', 'end': 'end_s', 'v': 'station'},
    },
    'ctrl_lidar/ctrl_ground.py': {
        'estimate_ground_plane': {'rng': 'random_generator', 'idx': 'sample_indices',
                                  'best': 'best_inliers', 'count': 'best_inlier_count'},
    },
    'ctrl_lidar/ctrl_free_space.py': {
        'ray_box_evidence': {'safe': 'safe_direction', 'a': 'lower_intersections',
                             'b': 'upper_intersections', 'near': 'entry_parameter',
                             'far': 'exit_parameter', 'remaining': 'distance_beyond_box'},
        'body_clearance': {'c': 'cos_yaw', 's': 'sin_yaw', 'local': 'local_points',
                           'o': 'local_origin', 'lo': 'cell_lower', 'hi': 'cell_upper',
                           'xi': 'x_cell', 'yi': 'y_cell', 'zi': 'z_cell'},
        'lane_patch_clearance': {'c': 'cos_yaw', 's': 'sin_yaw', 'local': 'local_points',
                                 'o': 'local_origin', 'distance2': 'distance_squared',
                                 'radius2': 'radius_squared', 'u': 'cell_x', 'v': 'cell_y'},
    },
    'ctrl_planning/ctrl_signal_stop.py': {
        'SignalStopPolicy.__init__': {'line': 'stop_line'},
        'SignalStopPolicy.current': {'line': 'stop_line', 'key': 'intersection_id',
                                     'x': 'candidate_line'},
        'SignalStopPolicy.lateral_hold_s': {'line': 'stop_line', 'x': 'candidate_line'},
        'SignalStopPolicy.observe': {'line': 'stop_line'},
        'SignalStopPolicy.apply': {'line': 'stop_line', 'stop': 'front_stop_s'},
    },
    'ctrl_ros1/ctrl_executors.py': {
        'Executor._next': {'ready': 'ready_entities', 'remaining': 'remaining_timeout',
                           'delay': 'wait_seconds', 'duration': 'timer_remaining_ns'},
        'MultiThreadedExecutor._check_futures': {'unfinished': 'pending_futures'},
    },
    'ctrl_ros1/ctrl_entities.py': {
        'Timer.take': {'ahead': 'overdue_ns'},
    },
}


def source_path(relative):
    package, filename = relative.split('/', 1)
    return REPO / 'ros1-port' / 'src' / package / package / filename


def scoped_names(tree):
    """Name 노드를 정의 범위별로 반환한다. lambda 인자명과 속성명은 보존한다."""
    class Visitor(ast.NodeVisitor):
        def __init__(self):
            self.scope = []
            self.names = {}

        def visit_definition(self, node):
            self.scope.append(node.name)
            self.generic_visit(node)
            self.scope.pop()

        visit_ClassDef = visit_definition
        visit_FunctionDef = visit_definition
        visit_AsyncFunctionDef = visit_definition

        def visit_Name(self, node):
            self.names.setdefault('.'.join(self.scope), []).append(node)

    visitor = Visitor()
    visitor.visit(tree)
    return visitor.names


def check():
    identifier_count = 0
    for relative, scopes in RENAMES.items():
        path = source_path(relative)
        current_git_path = path.relative_to(REPO).as_posix()
        git_path={v:k for k,v in MAP['paths'].items()}.get(current_git_path,current_git_path)
        baseline = subprocess.check_output(['git', 'show', BASE + ':' + git_path], cwd=REPO).decode('utf-8')
        before = ast.parse(baseline)
        after = legacy_ast(path.read_text(encoding='utf-8'))
        before_names = scoped_names(before)
        after_names = scoped_names(after)
        restored = copy.deepcopy(after)
        restored_names = scoped_names(restored)
        definitions = {node.name: node for node in ast.walk(before) if isinstance(node, ast.FunctionDef)}
        for scope, mapping in scopes.items():
            assert len(set(mapping.values())) == len(mapping), (relative, scope, 'duplicate destination')
            names = {node.id for node in before_names[scope]}
            stored = {node.id for node in before_names[scope] if isinstance(node.ctx, ast.Store)}
            function = definitions[scope.rsplit('.', 1)[-1]]
            parameters = {node.arg for node in ast.walk(function.args) if isinstance(node, ast.arg)}
            assert not (parameters & mapping.keys()), (relative, scope, 'public parameter')
            assert mapping.keys() <= stored, (relative, scope, 'not a local binding')
            assert not (names & set(mapping.values())), (relative, scope, 'name collision')
            inverse = {new: old for old, new in mapping.items()}
            assert not ({node.id for node in after_names[scope]} & mapping.keys()), (relative, scope, 'old reference')
            for node in restored_names[scope]:
                node.id = inverse.get(node.id, node.id)
            identifier_count += len(mapping)
        assert ast.dump(before) == ast.dump(restored), (relative, 'AST differs beyond approved local names')
        compile(after, str(path), 'exec')
    print(f'PASS: {len(RENAMES)} runtime files, {identifier_count} scoped local identifiers; inverse AST and syntax match {BASE}')


if __name__ == '__main__':
    check()
