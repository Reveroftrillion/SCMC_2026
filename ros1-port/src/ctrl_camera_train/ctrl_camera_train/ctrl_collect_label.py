"""조건 이름 하나로 1회 수집 → Ctrl+C → 해당 세션 자동 라벨 생성.

    ./tools/collect_label.sh foggy_13h

MORAI 날씨·시간은 직접 설정한다. 카메라 자세와 라벨 임계값은 바꾸지 않는다.
"""

import argparse
from collections import Counter
from datetime import datetime
import fcntl
import json
import math
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import tempfile


def condition_name(value):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,79}', value):
        raise argparse.ArgumentTypeError('이름은 영문·숫자로 시작하고 영문·숫자·_·-만 쓰세요 (최대 80자).')
    return value


def positive_speed(value):
    speed = float(value)
    if not math.isfinite(speed) or speed <= 0:
        raise argparse.ArgumentTypeError('속도는 0보다 큰 유한한 숫자여야 합니다.')
    return speed


def nonnegative_seconds(value):
    seconds = float(value)
    if not math.isfinite(seconds) or seconds < 0:
        raise argparse.ArgumentTypeError('정차 시간은 유한한 0 이상의 숫자여야 합니다.')
    return seconds


def run_child(command, *, collecting=False):
    """Forward one stop request to the launch parent; wait for its shutdown/flush.

    A new session prevents terminal Ctrl+C from also reaching every ROS child.
    ros2 launch itself forwards SIGINT and performs its normal shutdown sequence.
    SIGINT during collection means finish and label; during labeling it cancels.
    SIGTERM cancels the workflow at either stage.
    """
    child = None
    interrupted = False
    cancelled = False
    sent_stop = False

    def stop(signum, _frame):
        nonlocal interrupted, cancelled, sent_stop
        interrupted = True
        cancelled = cancelled or signum == signal.SIGTERM or not collecting
        print('\n종료 요청을 보냈습니다. 프로세스 종료와 저장 완료를 기다립니다.', flush=True)
        if child is not None and child.poll() is None and not sent_stop:
            try:
                child.send_signal(signal.SIGINT)
                sent_stop = True
            except ProcessLookupError:
                pass

    old_handlers = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        child = subprocess.Popen(command, start_new_session=True)
        if interrupted:
            stop(signal.SIGTERM if cancelled else signal.SIGINT, None)
        code = child.wait()
        if cancelled:
            raise KeyboardInterrupt
        if code != 0 and not (interrupted and code in (130, -signal.SIGINT)):
            phase = '수집' if collecting else '라벨 생성'
            raise RuntimeError(f'{phase} 실패 (종료 코드 {code}). 원본 사진은 보존했습니다.')
    finally:
        # Restore the terminal handler only after the child has finished.
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)


def new_session(root, before, tag):
    candidates = [p for p in root.iterdir()
                  if p not in before and p.is_dir() and (p / 'meta.csv').is_file()
                  and len(p.name.split('_', 2)) == 3 and p.name.split('_', 2)[2] == tag]
    if len(candidates) != 1:
        raise RuntimeError(f'이번 실행의 수집 세션을 하나로 특정할 수 없습니다 ({len(candidates)}개). '
                           f'원본 폴더를 확인하세요: {root}')
    return candidates[0]


def output_summary(out, session, tag):
    frames = json.loads((out / 'frames.json').read_text())
    classes = Counter(b['cls'] for f in frames for b in f['boxes'] if b['cls'])
    splits = {}
    for split in ('train', 'val'):
        labels = list((out / 'dataset/labels' / split).glob('*.txt'))
        splits[split] = {'images': len(labels),
                         'with_labels': sum(bool(p.read_text().strip()) for p in labels)}
    summary = {'status': 'labels_generated', 'tag': tag, 'session': str(session),
               'frames': len(frames), 'classes': dict(classes), 'splits': splits,
               'review_images': len(list((out / 'review').glob('*.jpg'))),
               'needs_manual_review': True}
    (out / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
    print(f'\n자동 라벨 초안 완료: {len(frames)}장 / 박스 {sum(classes.values())}개', flush=True)
    print(f'원본: {session}\n결과: {out}\n검수: {out / "review"}', flush=True)
    print('다음 단계: 박스·클래스 검수 → Roboflow 50장 시험 왕복. 아직 학습용 확정본이 아닙니다.')
    if not classes:
        print('⚠ 라벨 박스가 없습니다. 신호등이 보이는 구간을 수집했는지 확인하세요.')
    if not splits['val']['with_labels']:
        print('⚠ val에 라벨이 있는 사진이 없습니다. 다른 조건의 정상 세션과 합쳐 재분할해야 합니다.')


def collect_and_label(args):
    root = args.data_root.expanduser().resolve()
    out_root = args.out_root.expanduser().resolve()
    if not shutil.which('ros2'):
        raise RuntimeError('ros2를 찾을 수 없습니다. tools/collect_label.sh로 실행하세요.')
    root.mkdir(parents=True, exist_ok=True)
    # Prevent two instances of this helper from competing for the same camera ports.
    with (root / '.collect_label.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError('같은 저장 폴더에서 수집 프로그램이 이미 실행 중입니다.') from exc
        before = set(root.iterdir())
        print(f'조건: {args.tag}\nMORAI의 실제 날씨·시간이 이 이름과 일치해야 합니다.', flush=True)
        print(f'수집 시작 (자율주행={not args.no_drive}, 속도={args.speed_kmh:g} km/h). '
              '끝나면 Ctrl+C를 한 번 누르세요. 종료 후 자동 라벨링합니다.', flush=True)
        if not args.no_drive:
            print(f'각 등록 정지선에서 {args.stop_seconds:g}초 대기 '
                  '(0이면 정차 기능 꺼짐).', flush=True)
        with tempfile.TemporaryDirectory(prefix='foscar_camera_collect_') as status_dir:
            failure_file = Path(status_dir) / 'node_failures.txt'
            run_child(['ros2', 'launch', '--noninteractive', 'ctrl_camera_train', 'ctrl_record.launch.py',
                       f'tag:={args.tag}', f'out_dir:={root}', f'failure_file:={failure_file}',
                       f'drive:={str(not args.no_drive).lower()}',
                       f'stop_seconds:={args.stop_seconds}',
                       f'speed_kmh:={args.speed_kmh}', 'continuous:=true'], collecting=True)
            if failure_file.exists():
                raise RuntimeError('수집 노드가 비정상 종료했습니다. 자동 라벨링을 시작하지 않습니다.\n'
                                   + failure_file.read_text() + f'원본은 보존했습니다: {root}')
        session = new_session(root, before, args.tag)
        out = out_root / f'{datetime.now():%Y%m%d_%H%M%S%f}_{args.tag}'
        out.mkdir(parents=True, exist_ok=False)
        state = {'status': 'labeling', 'tag': args.tag, 'session': str(session)}
        status_file = out / 'summary.json'
        status_file.write_text(json.dumps(state, ensure_ascii=False, indent=2) + '\n')
        print(f'\n수집 종료. 이번 세션만 라벨링합니다: {session}', flush=True)
        try:
            run_child([sys.executable, '-u', '-m', 'ctrl_camera_train.ctrl_extract',
                       '--session', str(session), '--out', str(out), '--require-s'])
            output_summary(out, session, args.tag)
        except (Exception, KeyboardInterrupt) as exc:
            state.update(status='incomplete', error=str(exc) or '사용자 중단')
            status_file.write_text(json.dumps(state, ensure_ascii=False, indent=2) + '\n')
            print(f'미완료 결과 위치: {out}', file=sys.stderr)
            raise
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('tag', nargs='?', type=condition_name,
                        help='조건 이름. 생략하면 입력받음 (예: sunny_11h, foggy_13h)')
    parser.add_argument('--data-root', type=Path, default=Path('~/foscar_logs/dataset'),
                        help='원본 수집 폴더 (기본: ~/foscar_logs/dataset)')
    parser.add_argument('--out-root', type=Path, default=Path('out'),
                        help='자동 라벨 결과의 상위 폴더 (기본: out)')
    parser.add_argument('--speed-kmh', type=positive_speed, default=20.0)
    parser.add_argument('--stop-seconds', type=nonnegative_seconds, default=60.,
                        help='정지선마다 실제 정지 후 대기할 초 (기본 60, 0이면 끔)')
    parser.add_argument('--no-drive', action='store_true', help='제어 명령 없이 수집 (수동 주행용)')
    args = parser.parse_args(argv)
    try:
        if args.tag is None:
            try:
                args.tag = condition_name(input('조건 이름 (예: foggy_13h): ').strip())
            except argparse.ArgumentTypeError as exc:
                parser.error(str(exc))
        return collect_and_label(args)
    except (KeyboardInterrupt, EOFError):
        print('\n중단했습니다. 저장된 원본은 그대로 남아 있습니다.', file=sys.stderr)
        return 130
    except (OSError, RuntimeError, ValueError) as exc:
        print(f'오류: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
