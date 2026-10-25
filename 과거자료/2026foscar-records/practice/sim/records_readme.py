#!/usr/bin/env python3
"""Practice only: publish ~/foscar_logs/records.json to the GitHub main README (user 2026-09-29).

Only README.md is touched, in a detached worktree that follows origin/main (the local main branch has
diverged and is left alone). The table between the RECORDS markers is rewritten; nothing is committed
when it did not change. loop_runner calls this after every run; it can also be run by hand.

    python3 practice/sim/records_readme.py [--dry-run]
"""
import argparse
import json
import subprocess
from pathlib import Path

RECORDS = Path.home() / 'foscar_logs' / 'records.json'
LAPS = Path.home() / 'foscar_logs' / 'laps.json'      # every full lap (loop_runner)
WORKTREE = Path(__file__).resolve().parents[3] / '2026foscar-records'
BEGIN, END = '<!-- RECORDS:BEGIN -->', '<!-- RECORDS:END -->'
ORDER = ['SIGNAL_1', 'STATIC_OBSTACLE', 'PEDESTRIAN', 'SIGNAL_4', 'ROUNDABOUT', 'HIGHWAY', 'TUNNEL']
NAMES = {'lap': '풀랩', 'SIGNAL_1': '신호 1', 'STATIC_OBSTACLE': '정적 장애물(박스)', 'PEDESTRIAN': '보행자',
         'SIGNAL_4': '신호 4 (좌회전)', 'ROUNDABOUT': '회전교차로', 'HIGHWAY': '고속도로', 'TUNNEL': '터널'}


def fmt(seconds):
    m, s = divmod(float(seconds), 60.)
    return f'{int(m)}:{s:05.2f}' if m >= 1 else f'{s:.2f} s'


def table(data):
    rows = ['## 최고기록 (MORAI 시뮬, 자동 갱신)', '',
            '| 구간 | 최고기록 | 달성 시각 | 코드 |', '|---|---:|---|---|']
    missions = data.get('missions') or {}
    for key in ['lap'] + ORDER + sorted(k for k in missions if k not in ORDER):
        row = data.get('lap') if key == 'lap' else missions.get(key)
        if not row:
            continue
        rows.append(f"| {NAMES.get(key, key)} | {fmt(row['s'])} | {row.get('at', '')} | {row.get('code', '')} |")
    rows += ['', '미션 기록은 그 구간의 시작선부터 끝선까지를 통과했을 때만 인정한다 (중간 출발 시나리오 제외). '
             '신호 대기 시간이 포함될 수 있다.']
    try:
        laps = json.loads(LAPS.read_text()) if LAPS.is_file() else []
    except ValueError:
        laps = []
    if laps:
        rows += ['', '### 최근 풀랩 (출발~END 주행 시간, 최근 10회)', '',
                 '| 시각 | 랩타임 | 충돌 | 코드 |', '|---|---:|---:|---|']
        rows += [f"| {r.get('at', '')} | {fmt(r['s'])} | {r.get('collisions', 0)} | {r.get('code', '')} |"
                 for r in reversed(laps[-10:])]
    return '\n'.join(rows)


def git(*args, check=True):
    return subprocess.run(['git', '-C', str(WORKTREE), *args], check=check, capture_output=True, text=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()
    if not RECORDS.is_file():
        print('no records yet');return
    data = json.loads(RECORDS.read_text())
    if not WORKTREE.is_dir():
        raise SystemExit(f'missing worktree {WORKTREE}: git worktree add --detach ../2026foscar-records origin/main')
    git('fetch', '-q', 'origin', 'main')
    git('checkout', '-q', '--detach', 'origin/main')
    readme = WORKTREE / 'README.md'
    text = readme.read_text()
    block = f'{BEGIN}\n{table(data)}\n{END}'
    if BEGIN in text and END in text:
        new = text[:text.index(BEGIN)] + block + text[text.index(END) + len(END):]
    else:                                   # first time: right under the title block
        head, sep, rest = text.partition('\n\n')
        new = head + sep + block + '\n\n' + rest
    if new == text:
        print('README records unchanged');return
    if args.dry_run:
        print(block);return
    readme.write_text(new)
    git('add', 'README.md')
    git('commit', '-q', '-m', 'README: best records update\n\nCo-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>')
    push = git('push', 'origin', 'HEAD:main', check=False)
    print('README records pushed' if push.returncode == 0 else 'push failed: ' + push.stderr.strip())


if __name__ == '__main__':
    main()
