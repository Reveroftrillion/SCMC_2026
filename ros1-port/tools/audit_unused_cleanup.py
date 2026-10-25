#!/usr/bin/env python3
"""Read-only cleanup audit; reports go only to the explicitly selected directory.

Does not infer that an unreferenced file may be deleted. The reviewed deletion
allowlist is separate, and dynamic imports/CLI consumers require human review.
"""
import argparse
import ast
from collections import Counter, defaultdict
import csv
import hashlib
import json
from pathlib import Path
import subprocess

REPO = Path(__file__).resolve().parents[2]
MISSIONS = ('공통', '신호', '정적', '보행자', '회전교차로', '고속도로', '터널')


def source_files():
    names = subprocess.check_output(
        ['git', 'ls-files', '-c', '-o', '--exclude-standard', '-z'], cwd=REPO
    ).decode().split('\0')
    return sorted(name for name in set(names) if name and (REPO / name).is_file())


def modules_and_edges(names):
    modules = {}
    for name in names:
        parts = Path(name).parts
        if name.startswith('ros1-port/src/') and name.endswith('.py') and len(parts) >= 5:
            if parts[3].startswith('foscar_') and parts[4] != 'test':
                module = '.'.join(parts[3:])[:-3]
                if module.endswith('.__init__'):
                    module = module[:-len('.__init__')]
                modules[module] = name
    edges = defaultdict(set)
    for module, name in modules.items():
        tree = ast.parse((REPO / name).read_bytes())
        package = module if name.endswith('/__init__.py') else module.rpartition('.')[0]
        for node in ast.walk(tree):
            candidates = []
            if isinstance(node, ast.Import):
                candidates = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom):
                if node.level:
                    prefix = package.split('.')[:len(package.split('.')) - node.level + 1]
                    target = '.'.join(prefix + ([node.module] if node.module else []))
                else:
                    target = node.module or ''
                candidates = [target] + [target + '.' + alias.name for alias in node.names]
            for target in candidates:
                if target in modules:
                    edges[name].add(modules[target])
    # behavior_node imports these names from YAML/launch, with yield_ fallback.
    behavior = modules.get('ctrl_planning.ctrl_behavior_node')
    if behavior:
        for state in ('cruise', 'avoid', 'yield_', 'tunnel'):
            edges[behavior].add(modules['ctrl_planning.states.' + state])
    return modules, edges


def mission_tags(names):
    modules, edges = modules_and_edges(names)
    roots = {
        '공통': ['ctrl_control.ctrl_bridge', 'ctrl_control.ctrl_localization', 'ctrl_control.ctrl_follower'],
        '신호': ['ctrl_camera.ctrl_yolo_signal_node', 'ctrl_camera.ctrl_camera_bridge',
               'ctrl_planning.ctrl_signal_stop'],
        '정적': ['ctrl_planning.states.ctrl_avoid', 'ctrl_planning.ctrl_static_course'],
        '보행자': ['ctrl_planning.states.ctrl_yield'],
        '회전교차로': ['ctrl_planning.ctrl_roundabout_planner', 'ctrl_planning.ctrl_circular_gap'],
        '고속도로': ['ctrl_planning.ctrl_highway_runtime'],
        '터널': ['ctrl_planning.states.ctrl_tunnel'],
    }
    tags = defaultdict(set)
    for mission, seeds in roots.items():
        pending = [modules[m] for m in seeds if m in modules]
        seen = set()
        while pending:
            name = pending.pop()
            if name in seen:
                continue
            seen.add(name)
            tags[name].add(mission)
            pending.extend(edges[name])
    return tags, edges


def classify(name, tags):
    path = Path(name)
    text = name.lower()
    detected = set(tags.get(name, ()))
    for words, mission in [
        (('highway',), '고속도로'), (('tunnel',), '터널'),
        (('roundabout', 'circular_gap', 'ring_', 'entry_gate'), '회전교차로'),
        (('pedestrian', 'yield', 'deer'), '보행자'),
        (('static_', 'avoid'), '정적'),
        (('signal', 'stopline', 'stop_line', 'yolo', 'camera'), '신호'),
    ]:
        if any(word in text for word in words):
            detected.add(mission)
    runtime = name.startswith('ros1-port/src/') and '/test/' not in name
    runtime = runtime and path.suffix not in ('.md', '.png', '.svg')
    if runtime:
        if not detected:
            detected.add('공통')
        if path.name in ('CMakeLists.txt', 'package.xml', 'setup.py') or '/resource/' in name:
            role = '빌드·설치 계약 보존'
        elif name.endswith('.launch.py'):
            role = 'ROS2 비교·패키징 검증; ROS1 직접 실행 아님'
        elif 'camera_train' in name:
            role = '선택 데이터 수집·학습 도구; 기본 주행 미사용'
        elif '/scripts/' in name or name.endswith('.launch'):
            role = 'ROS1 실행/선택 기능 진입점'
        else:
            role = '공유 모듈·설정·지도·경로·모델; 선택/fallback 포함'
        category = next(m for m in MISSIONS if m in detected)
    else:
        category = '개발·검증'
        if name.startswith('2026_CTRL_competition/'):
            role = '보존 원본·출처·비교·과거 재현'
        elif name.startswith('과거자료/'):
            role = '과거 개발·계획·참고 자료; 원문 보존, 옛 실행 경로는 보관 안내 참조'
        elif name.startswith(('2026foscar/', '2026foscar-team2/', '2026foscar-records/')):
            role = '보류: 버전 차이·고유 시험·자기 트리 재현 의존성'
        elif name.startswith('sim_2025/'):
            role = '과거 참고·출처/오프라인 도구; 동적/외부 호출 미확인'
        elif name.startswith(('tunnel_tools/', '측정테스트파일/')):
            role = '과거 분석·측정 재현; 경로·ROS2 환경 미확인'
        elif name.startswith('Readme, 각종정보파일/') or name.startswith('global_path'):
            role = '제공 사양·지도·경로 비교 근거'
        elif '/test/' in name or path.name.startswith('test_'):
            role = '회귀 검사·fixture'
        elif '/docs/' in name or path.suffix == '.md':
            role = '안내·기술 문서·실험 근거 보존'
        else:
            role = '개발·빌드·검증·수집 도구/설정'
    return category, ', '.join(m for m in MISSIONS if m in detected) or '공통', role


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    before = {row['path']: row for row in json.loads(args.baseline.read_text())}
    names = source_files()
    tags, edges = mission_tags(names)
    rows = []
    for name in names:
        p = REPO / name
        category, missions, role = classify(name, tags)
        rows.append(dict(path=name, category=category, missions=missions, role=role,
                         bytes=p.lstat().st_size,
                         sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
    after = {row['path']: row for row in rows}
    summary = dict(
        before_files=len(before), before_bytes=sum(r['bytes'] for r in before.values()),
        after_files=len(after), after_bytes=sum(r['bytes'] for r in rows),
        deleted=[before[n] for n in sorted(before.keys() - after.keys())],
        added=sorted(after.keys() - before.keys()),
        modified=[n for n in sorted(before.keys() & after.keys())
                  if before[n]['sha256'] != after[n]['sha256']],
        categories=dict(Counter(r['category'] for r in rows)),
    )
    (args.output / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
    (args.output / 'import-edges.json').write_text(json.dumps(
        {k: sorted(v) for k, v in sorted(edges.items())}, ensure_ascii=False, indent=2) + '\n')
    with (args.output / 'retained-files.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(json.dumps({k: v for k, v in summary.items() if k != 'deleted'}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
