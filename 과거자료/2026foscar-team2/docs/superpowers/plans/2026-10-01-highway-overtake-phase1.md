# 고속도로 추월 1단계 (기준 측정 + B 원인 분석) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 현재 코드의 고속도로 구간 성능을 5회 측정해 기준을 만들고, 차선 변경 대기 중 "목표 차로 차 앞으로 들어가는 계획"이 어떤 검사에서 떨어지는지 기록 데이터로 센다.

**Architecture:** 슬롯 계획기(`plan_slot`)에 선택적 `why` 집계(기본 꺼짐, 동작 불변)를 넣고, 정책(`HighwayPolicy`)이 플래그가 켜졌을 때만 그 집계를 진단에 싣는다. `practice/`의 재생 도구가 기록된 bag의 대기 순간을 현재 정책으로 다시 돌려 이유를 모은다. B·A의 실제 수정은 이 분석 결과로 2단계 계획을 따로 쓴다.

**Tech Stack:** Python 3.10, ROS 2 Humble(rosbag2_py), numpy, pytest, MORAI 시뮬 + `practice/sim/loop_runner.py`.

**Spec:** `docs/superpowers/specs/2026-10-01-highway-overtake-design.md`

## Global Constraints

- 작업 폴더·브랜치: `~/Desktop/2026Morai/2026foscar-team2`, 브랜치 `team2`. 다른 워크트리는 읽기만 한다. push 하지 않는다.
- 주행 코드(`src/`)는 `practice/`를 import·참조하지 않는다. 새 도구는 `practice/` 안에 만들고 `bash practice/check_submission.sh` 가 exit 0 이어야 한다.
- 기존 안전 규칙(`pass_first`, `pass_rear_min_gap_m`, `rear_min_gap_m`, `REAR_CLOSING`, `follow_brake_mps2`)은 바꾸지 않는다. 이 단계는 동작을 바꾸지 않는다.
- 단위 테스트는 종료코드로 판정한다(파이프로 가리지 않는다). 실행 전 `source install/setup.bash`. `PYTHONNOUSERSITE=1` 은 쓰지 않는다.
- 기준 실패 14개(아래)만 허용한다. 새 실패가 생기면 안 된다.

```
src/foscar_control/test/test_mission_geometry.py::test_competition_highway_end_is_cp13_and_general_before_tunnel
src/foscar_planning/test/test_empty_entry.py::test_vehicle_gap_decision_releases_while_approaching_but_wait_still_stops
src/foscar_planning/test/test_entry_gate.py::test_distant_approach_does_not_crawl_at_staging_speed
src/foscar_planning/test/test_entry_gate.py::test_fixed_gps_waiting_point_overrides_old_entry_and_dynamic_stage
src/foscar_planning/test/test_entry_gate.py::test_ring_not_visible_yet_holds_speed_until_the_last_stop_point
src/foscar_planning/test/test_following_departure.py::test_point_six_launch_measured_progress_fits_updated_envelope
src/foscar_planning/test/test_highway_boxes.py::test_a_slow_lane_four_car_at_entry_is_passed_and_cut_in_ahead_of
src/foscar_planning/test/test_highway_curve_speed.py::test_cp12_approach_keeps_straight_speed_but_limits_the_actual_bend
src/foscar_planning/test/test_highway_policy.py::test_stopped_at_a_change_deadline_still_changes_once_the_lane_clears
src/foscar_planning/test/test_highway_policy.py::test_stopped_at_the_deadline_a_stopped_queue_beside_our_rear_does_not_deadlock_us
src/foscar_planning/test/test_highway_policy.py::test_change_curve_is_judged_on_the_change_not_the_bend_its_tail_runs_into
src/foscar_planning/test/test_highway_route.py::test_lane_centers_follow_map_and_rejoin_original
src/foscar_planning/test/test_highway_route.py::test_rejoin_and_highway_end_are_distinct_checkpoints
src/foscar_planning/test/test_mission_node.py::test_completed_entry_shows_exit_not_stale_front_following
```

전체 테스트 명령(저장소 최상위에서):

```bash
source install/setup.bash
python3 -m pytest -q -p no:cacheprovider src/foscar_control/test src/foscar_planning/test > /tmp/claude-1000/-home-foscar-Desktop-2026Morai/1a208d90-ae95-45f9-80f3-ee316282a4a0/scratchpad/pt.log 2>&1; rc=$?
grep -E "^FAILED|passed|failed" /tmp/claude-1000/-home-foscar-Desktop-2026Morai/1a208d90-ae95-45f9-80f3-ee316282a4a0/scratchpad/pt.log | tail -20; echo rc=$rc
```
기대: `14 failed, N passed`, FAILED 목록이 위 14개와 같다.

---

### Task 1: 고속도로 구간 요약 도구 + 기준 5회 측정

**Files:**
- Create: `practice/sim/analysis/highway_summary.py`

**Interfaces:**
- Consumes: `practice/sim/runs/<stamp>/results.jsonl`(러너 결과, `splits` 키 `'1118.7'`=CP10, `'1741.7'`=CP13), `practice/sim/pass_records.jsonl`(러너가 매 주행 뒤 추가; `kind` decision/change/contact, `run_dir`, `run`, `lanes`, `v_kmh`, `reason`).
- Produces: `summarize(run_dir) -> list[dict]`(키: `run, result, hw_s, collisions, contacts, wait_min_kmh, reasons`)와 CLI 출력. 2·3단계 비교에 같은 도구를 쓴다.

- [ ] **Step 1: 도구 작성**

```python
#!/usr/bin/env python3
"""PRACTICE ONLY: highway section summary of loop_runner runs (2026-10-01 overtake work, baseline vs changes).

Per run: CP10 -> CP13 time from the runner's splits, collisions, pass_log contacts, the lowest speed seen while
deciding each lane change (pass_records 'decision' rows, about one per second) and the decision reasons.

    python3 practice/sim/analysis/highway_summary.py practice/sim/runs/<stamp> [...]
"""
import json, os, statistics, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
CP10, CP13 = '1118.7', '1741.7'


def records(run_dir):
    name = os.path.basename(run_dir.rstrip('/'))
    return [r for r in map(json.loads, open(ROOT/'practice/sim/pass_records.jsonl')) if r.get('run_dir') == name]


def summarize(run_dir):
    rec = records(run_dir); rows = []
    for r in map(json.loads, open(os.path.join(run_dir, 'results.jsonl'))):
        sp = r.get('splits', {})
        hw = round(sp[CP13]-sp[CP10], 2) if CP10 in sp and CP13 in sp else None
        mine = [x for x in rec if x.get('run') == r['run']]
        waits = {}
        for x in mine:
            if x['kind'] == 'decision':waits.setdefault(x['lanes'], []).append(x['v_kmh'])
        rows.append(dict(run=r['run'], result=r['result'], hw_s=hw, collisions=len(r['collisions']),
                         contacts=sum(x['kind'] == 'contact' for x in mine),
                         wait_min_kmh={k: min(v) for k, v in sorted(waits.items(), reverse=True)},
                         reasons=sorted({x.get('reason') for x in mine if x['kind'] == 'decision'})))
    return rows


def main():
    rows = []
    for d in sys.argv[1:]:
        for row in summarize(d):
            row['dir'] = os.path.basename(d.rstrip('/'));rows.append(row)
            print(f"{row['dir']} run{row['run']} {row['result']:<24} CP10->CP13 {row['hw_s']} s  coll {row['collisions']} "
                  f"contact {row['contacts']}  wait min km/h {row['wait_min_kmh']}  {row['reasons']}")
    hw = [r['hw_s'] for r in rows if r['hw_s'] is not None]
    if hw:
        print(f'CP10->CP13: n={len(hw)} median {statistics.median(hw):.2f} s, min {min(hw):.2f}, max {max(hw):.2f}; '
              f"collisions {sum(r['collisions'] for r in rows)}, contacts {sum(r['contacts'] for r in rows)}")


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: 오늘 풀랩 두 개로 도구 확인**

Run: `python3 practice/sim/analysis/highway_summary.py practice/sim/runs/20261001_144548 practice/sim/runs/20261001_150014`
Expected: 두 줄 + 요약. `20261001_150014 run0 PASS CP10->CP13 37.4 s` (= `splits['1741.7']` 156.99 − `splits['1118.7']` 119.59). `wait_min_kmh`에 `4->3` 키가 있다.

- [ ] **Step 3: 시뮬 상태 확인 후 기준 5회**

다른 주행이 없는지: `ps aux | grep -E "loop_runner|ros2 launch" | grep -v grep | wc -l` → `0`.

Run (백그라운드, 약 6분):
```bash
source install/setup.bash && export PYTHONPATH=$PWD/.camera-deps:$PYTHONPATH
python3 practice/sim/loop_runner.py Practice3_CP08 --runs 5 --end-s 1745 --watch-from 1110 --stop-s 30 2>&1 | grep -E "results:|README"
```
Expected: `results: .../practice/sim/runs/<stamp>/results.jsonl`.

- [ ] **Step 4: 기준 요약**

Run: `python3 practice/sim/analysis/highway_summary.py practice/sim/runs/<stamp>`
Expected: 5줄 + 중앙값 줄. 결과(중앙값·최소·최대, 충돌·접촉 수, 단계별 최저 대기 속도)를 스펙 문서 끝 `## 측정 기록` 절에 `### 기준 (e10aa46, <stamp>)` 로 적는다.

- [ ] **Step 5: 제출 검사 후 커밋**

```bash
bash practice/check_submission.sh; echo rc=$?        # rc=0
git add practice/sim/analysis/highway_summary.py docs/superpowers/specs/2026-10-01-highway-overtake-design.md
git commit -m "practice: highway section summary; overtake baseline (5 x Practice3_CP08)

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```
(`practice/sim/pass_records.jsonl` 은 러너 기록이라 이 커밋에 넣지 않는다.)

---

### Task 2: `plan_slot` 거절 이유 집계 (`why`)

**Files:**
- Modify: `src/foscar_planning/foscar_planning/highway_slot_planner.py` (`plan_slot`, 73–225행)
- Test: `src/foscar_planning/test/test_highway_slot_why.py` (새 파일)

**Interfaces:**
- Produces: `plan_slot(..., why=None)`. `why`가 dict이면 후보(목표 속도 × 시작 시각)마다 `why[(mode, reason)] += 1`. `mode`는 `'ACCEL'|'KEEP'|'YIELD'`(Plan.mode와 같은 기준), `reason`은 다음 중 하나:
  - `'latest_start'` 시작점이 `change['latest_start']`를 넘음
  - `'deadline'` 곡선 끝이 `change['deadline']`을 넘음
  - `'horizon'` 진입·교차·이탈·끝 시점이 예측 창 밖
  - `'ease_late'` 감속이 변경 시작 전에 안 끝남
  - `'enter_late'` 차체 진입이 `max_enter_s` 넘게 늦음
  - `'lead'` 지금 차로 앞차 제동 여유 부족(시작 지연 후보라 감속 재판정 없음)
  - `'body'` 목표 차로 차의 앞에도 뒤에도 못 들어감(시작 지연 후보)
  - `'braked_lead'` / `'braked_body'` 지금 시작하며 감속해도 위와 같음
  - `'held_rear'` 착지 후 앞차 속도에 묶이면 뒤차가 너무 붙음
  - `'ok'` 허가된 후보
- `why=None`이면 동작·반환값이 지금과 완전히 같다.

- [ ] **Step 1: 실패하는 테스트 작성**

```python
"""plan_slot's optional rejection tally (2026-10-01 overtake analysis): which check refuses each candidate."""
import math
from pathlib import Path
from foscar_planning.highway_config import HighwayConfig
from foscar_planning.highway_policy import HighwayPolicy
from foscar_planning.highway_slot_planner import Body, plan_slot
from test_highway_boxes import highway_route


def planner():
    cfg = HighwayConfig.load(Path(__file__).parents[1]/'config/highway.yaml',
                             control_config=Path(__file__).parents[2]/'foscar_bringup/config/low_speed.yaml')
    return HighwayPolicy(highway_route(), cfg)._planner()


S = 1300.
CHANGE = dict(deadline=1500.)


def test_empty_lane_counts_accepted_candidates():
    why = {}
    plan = plan_slot(S, 20., [], None, planner(), change=CHANGE, why=why)
    assert plan is not None
    assert sum(n for (mode, reason), n in why.items() if reason == 'ok') > 0


def test_a_stopped_lead_just_ahead_refuses_every_candidate_for_the_lead():
    P = planner()
    lead = Body('lead', S+P.front+1., S+P.front+5., 0., 0., 'lead')
    why = {}
    assert plan_slot(S, 20., [], lead, P, change=CHANGE, why=why) is None
    reasons = {reason for (mode, reason) in why}
    assert 'ok' not in reasons
    assert reasons & {'lead', 'braked_lead'}


def test_the_tally_does_not_change_the_plan():
    P = planner()
    bodies = [Body('side', S-3., S+2., 18., 0.3, 'measured'), Body('ahead', S+30., S+34.5, 15., 0.3, 'measured')]
    lead = Body('lead', S+40., S+44.5, 12., 0.3, 'lead')
    assert plan_slot(S, 20., bodies, lead, P, change=CHANGE) == plan_slot(S, 20., bodies, lead, P, change=CHANGE, why={})
```

- [ ] **Step 2: 실패 확인**

Run: `source install/setup.bash && python3 -m pytest -q -p no:cacheprovider src/foscar_planning/test/test_highway_slot_why.py; echo rc=$?`
Expected: 3 failed, `TypeError: plan_slot() got an unexpected keyword argument 'why'`, rc=1.

- [ ] **Step 3: 구현**

`plan_slot` 시그니처와 docstring:

```python
def plan_slot(s0,v0,bodies,lead,P,change=None,taper=None,vmax=33.3,only=None,pass_first=True,why=None):
    """Best Plan or None.
    ...(기존 설명 그대로)...
    why: optional dict, counts each refused / accepted candidate as why[(mode, reason)] (analysis only, 2026-10-01).
    """
```

`for goal in goals:` 바로 아래(첫 줄 `x,v,tr=profile(...)` 앞)에 추가:

```python
        mode_g='ACCEL' if goal>v0+.3 else 'YIELD' if goal<v0-.3 else 'KEEP'
        def tally(reason):
            if why is not None:why[(mode_g,reason)]=why.get((mode_g,reason),0)+1
```

창 만들기 루프의 두 `break` 앞에 집계:

```python
                if start>change.get('latest_start',math.inf):tally('latest_start');break
                ...
                if start+length>change['deadline']:tally('deadline');break
```

후보 평가 루프(`for kind,t0,start,length,enter,cross,leave,done in windows:`)의 각 `continue`를 다음처럼 바꾼다:

```python
            if None in (ie,ic,il,idn) or idn+tail>=N:tally('horizon');continue
            if goal<v0-.3 and tr>t0+1e-9:tally('ease_late');continue              # ease off before the change starts, not while changing
            if kind=='CHANGE' and ts[ie]-t0>P.max_enter+1e-9:tally('enter_late');continue
            ...
            if lead_fail or (n and not np.all(r_ok|t_ok)):
                # Too close ahead at the planned speed: starting now, judge it with the speed shed to the safety cap.
                if kind!='CHANGE' or t0>0.:tally('lead' if lead_fail else 'body');continue
                xb=braked(i0,ie,il,idn,end)
                if xb is None:tally('braked_lead' if lead_fail else 'braked_body');continue
            ...
                        if np.any(gr<np.where(v_nom[rears]>ve,P.rear_gap,P.pass_rear_gap)):tally('held_rear');continue
```

`plan=Plan(...)` 줄 바로 뒤에 `tally('ok')`. 기존 `mode=...` 계산은 그대로 둔다(`mode_g`와 같은 값).

- [ ] **Step 4: 통과 확인**

Run: `python3 -m pytest -q -p no:cacheprovider src/foscar_planning/test/test_highway_slot_why.py; echo rc=$?`
Expected: `3 passed`, rc=0.

- [ ] **Step 5: 전체 테스트**

Global Constraints의 전체 테스트 명령. Expected: 14 failed(목록 동일), 새 테스트 3개 포함 통과 수 +3.

- [ ] **Step 6: 커밋**

```bash
git add src/foscar_planning/foscar_planning/highway_slot_planner.py src/foscar_planning/test/test_highway_slot_why.py
git commit -m "highway: plan_slot can tally why each slot candidate is refused (analysis only, off by default)

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 3: 정책이 플래그로 집계를 진단에 싣기

**Files:**
- Modify: `src/foscar_planning/foscar_planning/highway_policy.py` (`__init__` 36–40행, `wait_gap`의 첫 `plan_slot` 호출 597행 부근)
- Test: `src/foscar_planning/test/test_highway_slot_why.py` (추가)

**Interfaces:**
- Consumes: Task 2의 `plan_slot(..., why=...)`.
- Produces: `HighwayPolicy.trace_slots: bool`(기본 `False`, `reset()`이 건드리지 않음). `True`면 대기 판단 틱의 `out.diagnostics['slot_why']`가 `{'<mode>:<reason>': count}`. `False`면 키가 없다.

- [ ] **Step 1: 실패하는 테스트 추가**

```python
from test_highway_boxes import policy, drive, ego, car


def test_policy_reports_the_tally_only_when_asked(highway):
    e = ego(highway, s=1360., lane=4, v=20.)
    world = [car(highway, 4, 1360.+12., 'lead', v=8.), car(highway, 3, 1360.+1., 'side', v=20.)]
    quiet = policy(4); out = drive(quiet, e, world, ticks=4)
    assert 'slot_why' not in out.diagnostics
    traced = policy(4); traced.trace_slots = True; out = drive(traced, e, world, ticks=4)
    assert isinstance(out.diagnostics.get('slot_why'), dict) and out.diagnostics['slot_why']
    assert all(':' in k for k in out.diagnostics['slot_why'])
```
(`highway` 픽스처는 `test_highway_route.py`에 있다. 같은 import 방식으로 `from test_highway_route import highway` 를 파일 위에 추가한다.)

- [ ] **Step 2: 실패 확인**

Run: `python3 -m pytest -q -p no:cacheprovider src/foscar_planning/test/test_highway_slot_why.py::test_policy_reports_the_tally_only_when_asked; echo rc=$?`
Expected: FAIL(`slot_why` 없음), rc=1.

- [ ] **Step 3: 구현**

`__init__`(`self.route=...`·`self.config=...` 대입 뒤, `self.reset()` 호출 전)에:

```python
        self.trace_slots=False   # analysis only (practice/sim/slot_why_report.py): diagnostics['slot_why'] per waiting tick
```

`wait_gap`의 첫 호출을 다음으로 바꾼다:

```python
            why={} if self.trace_slots else None
            plan=plan_slot(ctx.s,max(0.,ctx.v),bodies,lead,self._planner(),change=change,taper=taper,vmax=vmax,pass_first=pass_first,why=why)
            if why is not None:diag['slot_why']={f'{m_}:{r_}':k_ for (m_,r_),k_ in sorted(why.items())}
```

(`only=` 재검사 호출은 바꾸지 않는다.)

- [ ] **Step 4: 통과 확인**

Run: `python3 -m pytest -q -p no:cacheprovider src/foscar_planning/test/test_highway_slot_why.py; echo rc=$?`
Expected: `4 passed`, rc=0.

- [ ] **Step 5: 전체 테스트**

Global Constraints의 전체 테스트 명령. Expected: 14 failed(목록 동일).

- [ ] **Step 6: 커밋**

```bash
git add src/foscar_planning/foscar_planning/highway_policy.py src/foscar_planning/test/test_highway_slot_why.py
git commit -m "highway: trace_slots puts the slot planner's refusal tally into the waiting diagnostics (off by default)

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 4: 재생 분석 도구 + B 원인 보고

**Files:**
- Create: `practice/sim/slot_why_report.py`
- Modify: `docs/superpowers/specs/2026-10-01-highway-overtake-design.md` (`## 측정 기록`에 `### B 원인 분석` 추가)

**Interfaces:**
- Consumes: `slot_replay.load()`, `slot_replay.snapshot(d)`, `cut_in_log.read_highway(bag)`(둘 다 `practice/sim/`), Task 3의 `HighwayPolicy.trace_slots`, `out.diagnostics['slot_why']`.
- Produces: 단계(`5->4`/`4->3`/`3->2`) × 기록된 결정(`NO_SLOT`/`SLOT_YIELD`)별 틱 수와 거절 이유 상위 목록. 2단계 계획의 근거.

- [ ] **Step 1: 도구 작성**

```python
#!/usr/bin/env python3
"""PRACTICE ONLY: why the slot planner found no slot ahead (2026-10-01 overtake work).

For every recorded waiting tick whose decision was NO_SLOT or SLOT_YIELD, replay the current HighwayPolicy with
trace_slots on and add up why each candidate plan was refused, per lane change and recorded decision. ACCEL / KEEP
rows are the plans that would have got ahead of a target-lane car.

    python3 practice/sim/slot_why_report.py practice/sim/runs/<stamp> [...] [--reasons NO_SLOT,SLOT_YIELD]
"""
import argparse, collections, glob, os, re, sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from slot_replay import load, snapshot                                  # noqa: E402
from cut_in_log import read_highway                                     # noqa: E402
from foscar_planning.highway_policy import HighwayPolicy                # noqa: E402
from foscar_planning.state import Context                               # noqa: E402


def logs(run_dir):
    for log in sorted(glob.glob(os.path.join(run_dir, 'launch_*.log'))):
        m = re.search(r'(run_[0-9_]+)_0\.db3', open(log, errors='ignore').read())
        if not m:continue
        bag = os.path.expanduser('~/foscar_logs/bags/'+m.group(1))
        if os.path.exists(bag):yield [d for d in read_highway(bag) if 'global_s' in d and d.get('ego_xy')]


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('runs', nargs='+'); ap.add_argument('--reasons', default='NO_SLOT,SLOT_YIELD')
    a = ap.parse_args(); want = set(a.reasons.split(','))
    cfg, road = load()
    ticks = collections.Counter(); why = collections.defaultdict(collections.Counter)
    for run in a.runs:
        for L in logs(run):
            lane = p = None
            for d in L:
                if d.get('state') != 'WAIT_GAP' or d.get('reason') not in want:continue
                if d.get('current_lane') in (None, road.lanes[-1]):continue
                if d['current_lane'] != lane:
                    p = HighwayPolicy(road, cfg); p.trace_slots = True; lane = d['current_lane']
                p.current_lane = lane; p.cp11 = bool(d.get('cp11_passed')); p.committed = None; p.previous = None
                ctx = Context(s=d['global_s'], x=d['ego_xy'][0], y=d['ego_xy'][1], yaw=d['ego_yaw'], v=d['ego_speed'],
                              now=d['stamp'], mission_name='HIGHWAY')
                out = p.tick(ctx, snapshot(d))
                key = f"{lane}->{road.lanes[road.lanes.index(lane)+1]} {d['reason']}"
                ticks[key] += 1; why[key].update(out.diagnostics.get('slot_why') or {})
    for key in sorted(ticks):
        print(f'{key}: {ticks[key]} ticks')
        for reason, n in why[key].most_common(14):print(f'    {reason:24s} {n}')


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: 기준 5회 + 오늘 풀랩으로 실행**

Run:
```bash
source install/setup.bash
python3 practice/sim/slot_why_report.py practice/sim/runs/<기준 stamp> practice/sim/runs/20261001_144548 practice/sim/runs/20261001_150014
```
Expected: `4->3 NO_SLOT: N ticks` 등 단계별 블록과 `ACCEL:lead`, `KEEP:body` 같은 이유 집계. bag 없는 회차는 건너뛴다.

- [ ] **Step 3: 오늘 competition 회차로 표본 늘리기 (읽기만)**

Run:
```bash
python3 practice/sim/slot_why_report.py $(ls -d ../2026foscar-competition/practice/sim/runs/20261001_*/)
```
Expected: 위와 같은 형식. (bag 이 남아 있는 회차만 집계된다.)

- [ ] **Step 4: 원인 정리**

스펙의 `## 측정 기록`에 `### B 원인 분석` 을 추가한다: 단계별 틱 수, ACCEL·KEEP 후보를 가장 많이 거절한 이유 상위 3개(개수), 그리고 그 이유가 스펙 2절의 수정 방향 두 가지(감속 후 속도 반영 / 빠를 때 미리 계획) 중 어느 쪽을 가리키는지 한 문단.

- [ ] **Step 5: 제출 검사 후 커밋**

```bash
bash practice/check_submission.sh; echo rc=$?        # rc=0
git add practice/sim/slot_why_report.py docs/superpowers/specs/2026-10-01-highway-overtake-design.md
git commit -m "practice: slot_why_report replays waiting ticks with the refusal tally; B cause analysis

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

- [ ] **Step 6: 사용자 보고 후 2단계 계획**

분석 결과를 사용자에게 수치로 보고하고, 선택한 B 수정 방향으로 `docs/superpowers/plans/2026-10-01-highway-overtake-phase2-b.md` 를 이 계획과 같은 형식으로 쓴다. A-1, A-2 계획은 B 결과 뒤에 각각 쓴다.
