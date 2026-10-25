# 고속도로 추월 2단계 A-1 (3→2 변경을 미뤄 앞지르기) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 3차로에서 2차로로 들어갈 때 2차로의 느린 차를 3차로로 앞질러 그 앞에 들어갈 수 있으면 그렇게 해서, 2차로에서 느린 차를 따라가는 시간을 줄인다.

**Architecture:** 슬롯 계획기(`plan_slot`)는 변경 시작 시각을 `max_start_s`(6 s)까지만 본다. 그래서 6 s 안에 못 앞지르는 2차로 느린 차는 "그 앞 슬롯"이 후보에 없고 뒤 슬롯만 남는다. `change` dict 에 선택적 `max_start` 를 받아 그 창을 넓히고, 정책은 마지막 변경(목표 차로 = `road.lanes[-1]`, 즉 3→2)에서만 설정값 `boxes.last_change_max_start_s` 를 넘긴다. 마감점을 넘는 시작은 계획기가 이미 `start+length>deadline` 에서 끊는다. 슬롯 선택(15 s 진행 거리 비교)과 안전 검사는 그대로다.

**Tech Stack:** Python 3.10, ROS 2 Humble, numpy, pytest, MORAI + `practice/sim/loop_runner.py`.

**Spec:** `docs/superpowers/specs/2026-10-01-highway-overtake-design.md` (§3 A-1, `## 측정 기록`)

## Global Constraints

- 작업 폴더·브랜치: `~/Desktop/2026Morai/2026foscar-team2`, 브랜치 `team2`. 다른 워크트리는 읽기만. push 하지 않는다.
- 주행 코드(`src/`)는 `practice/`를 import·참조·언급하지 않는다(주석 포함). `bash practice/check_submission.sh 2>&1 | grep -A6 "outside practice"` 에 기존 3줄(obstacle_node.py:730, race_hud_node.py:6, highway_box_tracks.py:35) 외 새 줄이 없어야 한다.
- 기존 안전 규칙(`pass_first`, `pass_rear_min_gap_m`, `rear_min_gap_m`, `REAR_CLOSING`, `follow_brake_mps2`)과 슬롯 선택 규칙(`progress_horizon_s`, `speed_cost_s`)은 바꾸지 않는다.
- 1차로는 쓰지 않는다(`excluded_lanes: [1]`, 흰 실선).
- 3→2 마감점 s1548.0(`inspect_highway_route.py`). 넘어서 끝나는 변경은 계획기가 이미 거른다.
- 테스트는 종료코드로 판정. `source install/setup.bash` 후 실행, `PYTHONNOUSERSITE` 금지. 테스트가 `build/` 사본을 import 하면 `colcon build --symlink-install --packages-select foscar_planning` 후 다시 돌린다.
- 기준 실패 14개만 허용(1단계 계획과 같은 목록): test_mission_geometry 1, test_empty_entry 1, test_entry_gate 3, test_following_departure 1, test_highway_boxes 1(`test_a_slow_lane_four_car_at_entry_is_passed_and_cut_in_ahead_of`), test_highway_curve_speed 1, test_highway_policy 3, test_highway_route 2, test_mission_node 1.
- 전체 테스트 명령:
  ```bash
  python3 -m pytest -q -p no:cacheprovider src/foscar_control/test src/foscar_planning/test > /tmp/claude-1000/-home-foscar-Desktop-2026Morai/1a208d90-ae95-45f9-80f3-ee316282a4a0/scratchpad/pt.log 2>&1; echo rc=$?
  grep -E "^FAILED|passed|failed" /tmp/claude-1000/-home-foscar-Desktop-2026Morai/1a208d90-ae95-45f9-80f3-ee316282a4a0/scratchpad/pt.log | tail -20
  ```
- 시뮬은 한 번에 하나만. 시작 전 `ps aux | grep -E "loop_runner|ros2 launch" | grep -v grep | wc -l` 가 0. 풀랩은 `--end-s 2165 --timeout 400` 필수.
- 커밋 끝줄: `Co-Authored-By: <작성 모델> <noreply@anthropic.com>`.

---

### Task 1: 측정 지표 추가 (CP10→CP12, 2차로 저속 추종 시간) + 기준 다시 계산

**Files:**
- Modify: `practice/sim/analysis/highway_summary.py`

**Interfaces:**
- Consumes: `practice/sim/analysis/cp_splits.py` 의 `splits(run_dir, n)`(체크포인트별 `(이름, 시각, 거리)` — 파일을 읽어 반환 형식을 확인할 것), `practice/sim/runs/<stamp>/launch_<n>.log` 안의 `log=/home/foscar/foscar_logs/follow_<...>.csv`(경로 추출), follow CSV 열 `elapsed_s,state,reason,...,speed_mps,s_m,...,target_speed_mps`.
- Produces: `summarize(run_dir)` 행에 키 2개 추가 — `cp10_cp12_s`(float|None), `lane2_follow_s`(float: CP12 전, 마지막 3→2 변경이 끝난 뒤 `reason=='HIGHWAY_FOLLOW'` 이고 속도 < 70 km/h 인 시간의 합). CLI 출력 줄과 요약 줄에도 두 값(중앙값)을 넣는다.

- [ ] **Step 1: `cp_splits.py` 의 `splits()` 반환 형식 확인**

Run: `sed -n 25,60p practice/sim/analysis/cp_splits.py`
Expected: 체크포인트 이름(`'CP10'`, `'CP12'` 형식인지 숫자인지), 시각, 거리의 순서를 확인한다. 아래 코드의 `cp_time` 을 그 형식에 맞춘다.

- [ ] **Step 2: 지표 코드 추가**

`highway_summary.py` 에 추가(기존 함수 유지):

```python
import csv, re, sys as _sys
_sys.path.insert(0, str(ROOT/'practice/sim/analysis'))
from cp_splits import splits as cp_splits                             # noqa: E402

LANE2_SLOW_KMH = 70.   # "stuck behind a slow lane-2 car": HIGHWAY_FOLLOW below this (lane 2 runs ~100 km/h when free)


def cp_time(run_dir, n, name):
    for row in cp_splits(run_dir, n):
        if str(row[0]) == name:return float(row[1])
    return None


def follow_csv(run_dir, n):
    m = re.search(r'log=(\S+follow_\S+\.csv)', open(os.path.join(run_dir, f'launch_{n}.log'), errors='ignore').read())
    return m.group(1) if m and os.path.exists(m.group(1)) else None


def lane2_follow_s(path, rejoin_s=1586.5):
    rows = list(csv.DictReader(open(path)))
    last_change = max((i for i, r in enumerate(rows) if r['reason'] in ('HIGHWAY_CHANGE', 'HIGHWAY_SETTLE')), default=None)
    if last_change is None:return 0.
    tot = 0.
    for a, b in zip(rows[last_change:], rows[last_change+1:]):
        if a['reason'] == 'HIGHWAY_FOLLOW' and float(a['s_m']) < rejoin_s and float(a['speed_mps'])*3.6 < LANE2_SLOW_KMH:
            tot += float(b['elapsed_s'])-float(a['elapsed_s'])
    return round(tot, 2)
```

`summarize()` 의 행 dict 에 추가:

```python
                         cp10_cp12_s=(lambda a, b: round(b-a, 2) if a is not None and b is not None else None)(
                             cp_time(run_dir, r['run'], 'CP10'), cp_time(run_dir, r['run'], 'CP12')),
                         lane2_follow_s=(lambda p: lane2_follow_s(p) if p else None)(follow_csv(run_dir, r['run'])),
```

(`'CP10'`/`'CP12'` 는 Step 1 에서 확인한 이름 형식으로 맞춘다.) `main()` 의 행 출력에 `CP10->CP12 {cp10_cp12_s} s  lane2 slow follow {lane2_follow_s} s` 를 넣고, 요약 줄 아래에 두 값의 중앙값 줄을 추가한다(값이 None 인 행은 제외).

- [ ] **Step 3: 기준 회차로 확인**

Run:
```bash
python3 practice/sim/analysis/highway_summary.py practice/sim/runs/20261001_155108 practice/sim/runs/20261001_144548 practice/sim/runs/20261001_150014
```
Expected: 7줄 + 요약. `20261001_150014 run0` 의 `lane2 slow follow` 가 0 보다 크다(그 랩은 s1460–1593 을 60 km/h 안팎으로 따라갔다). 숫자를 스펙 `## 측정 기록` 아래 `### A-1 기준 지표` 로 적는다(회차별 CP10→CP12, lane2 slow follow, 중앙값).

- [ ] **Step 4: 커밋**

```bash
bash practice/check_submission.sh 2>&1 | grep -A6 "outside practice"     # 기존 3줄만
git add practice/sim/analysis/highway_summary.py docs/superpowers/specs/2026-10-01-highway-overtake-design.md
git commit -m "practice: highway summary reports CP10->CP12 and slow lane-2 following; A-1 baseline numbers"
```

---

### Task 2: `plan_slot` 이 `change['max_start']` 로 시작 창을 넓힌다

**Files:**
- Modify: `src/foscar_planning/foscar_planning/highway_slot_planner.py` (`plan_slot`)
- Test: `src/foscar_planning/test/test_highway_slot_why.py` 옆에 새 파일 `src/foscar_planning/test/test_highway_last_change.py`

**Interfaces:**
- Produces: `change` dict 의 선택 키 `max_start`(float, s). 없으면 `P.max_start` 를 그대로 쓴다(동작 불변). 있으면 이 값이 (1) 시간 창 `H`, (2) 목표 속도 상한 `hi` 계산, (3) 시작 시각 후보 `np.arange(0., max_start+1e-9, P.start_step)` 세 곳의 `P.max_start` 를 대신한다. `taper`·`only` 경로는 그대로.

- [ ] **Step 1: 실패하는 테스트 작성**

```python
"""A-1 (2026-10-01): the last change (3->2) may wait longer than max_start_s to pass a slow lane-2 car and land ahead of it."""
from pathlib import Path
from foscar_planning.highway_config import HighwayConfig
from foscar_planning.highway_policy import HighwayPolicy
from foscar_planning.highway_slot_planner import Body, plan_slot
from test_highway_boxes import highway_route


def planner():
    cfg = HighwayConfig.load(Path(__file__).parents[1]/'config/highway.yaml',
                             control_config=Path(__file__).parents[2]/'foscar_bringup/config/low_speed.yaml')
    return HighwayPolicy(highway_route(), cfg)._planner()


S = 1000.
V0 = 20.          # ego, m/s
SLOW = 12.        # lane-2 car, m/s
AHEAD = 95.       # its tail this far ahead of our rear axle: too far to pass within max_start_s 6 s


def slow_car():
    return Body('slow', S+AHEAD, S+AHEAD+4.6, SLOW, .3, 'measured')


def test_within_max_start_the_slow_car_far_ahead_is_landed_behind():
    plan = plan_slot(S, V0, [slow_car()], None, planner(), change=dict(deadline=S+400.))
    assert plan is not None and plan.front == 'slow' and plan.rear == ''


def test_a_longer_start_window_passes_it_and_lands_ahead():
    plan = plan_slot(S, V0, [slow_car()], None, planner(), change=dict(deadline=S+400., max_start=15.))
    assert plan is not None and plan.rear == 'slow' and plan.front == ''
    assert plan.start_t > 6.


def test_the_deadline_still_bounds_a_long_window():
    plan = plan_slot(S, V0, [slow_car()], None, planner(), change=dict(deadline=S+150., max_start=15.))
    assert plan is None or plan.start_s+plan.length_m <= S+150.+1e-6


def test_no_max_start_key_is_the_old_behaviour():
    P = planner(); ch = dict(deadline=S+400.)
    assert plan_slot(S, V0, [slow_car()], None, P, change=ch) == plan_slot(S, V0, [slow_car()], None, P, change={**ch, 'max_start': P.max_start})
```

`AHEAD`/`SLOW`/`V0` 은 "6 s 안에는 못 앞지르고 15 s 안에는 앞지른다" 가 되도록 고른 값이다. Step 2 에서 첫 테스트가 기대와 다르게 나오면(예: 6 s 안에 이미 앞지름), `AHEAD` 를 60–100 m, `SLOW` 를 8–15 m/s 범위에서 조정해 첫 두 테스트의 전제가 성립하게 하고, 고른 값과 이유를 보고서에 적는다. 단언(assert)은 약하게 바꾸지 않는다.

- [ ] **Step 2: 실패 확인**

Run: `python3 -m pytest -q -p no:cacheprovider src/foscar_planning/test/test_highway_last_change.py; echo rc=$?`
Expected: `test_a_longer_start_window_passes_it_and_lands_ahead` FAIL(창이 6 s 라 뒤 슬롯), 나머지 PASS. rc=1.

- [ ] **Step 3: 구현**

`plan_slot` 맨 앞(`dt=P.dt;H=...` 줄)을 다음으로 바꾼다:

```python
    max_start=change.get('max_start',P.max_start) if change is not None else P.max_start
    dt=P.dt;H=max_start+max(12.,P.change_time*4)+P.tail_s
```

목표 속도 상한 줄:

```python
        lo=max(1.,v0-10.);hi=max(lo,min(vmax,v0+P.accel*(max_start+P.change_time)))
```

시작 시각 후보 줄:

```python
            starts=[0.] if only is not None else np.arange(0.,max_start+1e-9,P.start_step)
```

docstring 의 `change:` 설명에 `, max_start= (optional: start times tried up to this, default P.max_start)` 를 덧붙인다.

- [ ] **Step 4: 통과 확인 + 전체 테스트**

Run: `python3 -m pytest -q -p no:cacheprovider src/foscar_planning/test/test_highway_last_change.py; echo rc=$?` → 4 passed, rc=0.
그다음 Global Constraints 의 전체 테스트 명령 → 14 failed(목록 동일).

- [ ] **Step 5: 커밋**

```bash
git add src/foscar_planning/foscar_planning/highway_slot_planner.py src/foscar_planning/test/test_highway_last_change.py
git commit -m "highway: plan_slot takes an optional longer start window per change (change['max_start'])"
```

---

### Task 3: 정책이 마지막 변경(3→2)에서만 긴 시작 창을 쓴다 + 판단 시간 측정

**Files:**
- Modify: `src/foscar_planning/config/highway.yaml` (`boxes:` 의 `max_start_s` 바로 아래)
- Modify: `src/foscar_planning/foscar_planning/highway_config.py` (boxes 키 검증이 목록을 쓰면 새 키 추가)
- Modify: `src/foscar_planning/foscar_planning/highway_policy.py` (`wait_gap` 의 `change=dict(...)` 588행 부근)
- Test: `src/foscar_planning/test/test_highway_last_change.py` (추가)

**Interfaces:**
- Consumes: Task 2 의 `change['max_start']`.
- Produces: `boxes.last_change_max_start_s`(yaml). 정책은 `target_lane==r.lanes[-1]` 이고 `entry` 가 아닐 때 `change['max_start']=b.last_change_max_start_s` 를 넣는다. 그 밖의 변경은 지금과 같다.

- [ ] **Step 1: 설정값 추가**

`highway.yaml` `max_start_s: 6.0` 줄 바로 아래:

```yaml
  last_change_max_start_s: 15.0      # ...and for the last change (3->2) up to this: wait in lane 3 to pass a slow lane-2 car
                                     # and land ahead of it (A-1, 2026-10-01). 15 s ~ lane 3 from the 4->3 landing (~s1300) to
                                     # the 3->2 deadline s1548 at 60 km/h; past the deadline the planner drops a start anyway.
```

`highway_config.py` 에서 boxes 키를 검증하거나 dataclass 로 읽는다면 같은 방식으로 `last_change_max_start_s` 를 추가한다(`grep -n "max_start_s" src/foscar_planning/foscar_planning/highway_config.py` 로 위치 확인).

- [ ] **Step 2: 실패하는 정책 테스트 추가**

```python
from test_highway_route import highway
from test_highway_boxes import policy, drive, ego, car


def test_waiting_in_lane_3_the_policy_plans_past_a_slow_lane_2_car(highway):
    s = 1330.
    e = ego(highway, s=s, lane=3, v=V0)
    slow = car(highway, 2, s+AHEAD, 'slow', v=SLOW)
    p = policy(3); drive(p, e, [slow], ticks=4)
    assert p.plan is not None and p.plan.rear != '' and p.plan.front == '', p.plan
```

(`policy(3)` 의 목표 차로는 2 = `road.lanes[-1]`. 첫 두 틱은 속도 측정 전이라 4틱을 돌린다. 이 테스트가 Task 2 의 값으로 전제가 안 맞으면 `s` 를 1300–1360 에서 조정하고 이유를 보고서에 적는다.)

Run: `python3 -m pytest -q -p no:cacheprovider src/foscar_planning/test/test_highway_last_change.py; echo rc=$?` → 새 테스트 FAIL, rc=1.

- [ ] **Step 3: 구현**

`wait_gap` 의 `else:change=dict(deadline=end-1. if late else deadline)` 다음 줄에:

```python
            if not entry and target_lane==r.lanes[-1]:change['max_start']=self.config.boxes.last_change_max_start_s
```

- [ ] **Step 4: 통과 확인 + 전체 테스트**

`test_highway_last_change.py` 5 passed. 전체 테스트 14 failed(목록 동일).

- [ ] **Step 5: 판단 시간 측정**

기록된 bag 을 현재 정책으로 재생해 틱당 판단 시간을 잰다(시뮬 불필요). `slot_replay.py --set` 은 `boxes` 키를 덮어쓰므로 "전" 은 창을 6 s 로 되돌린 값이다.

```bash
python3 practice/sim/slot_replay.py practice/sim/runs/20261001_150014 practice/sim/runs/20261001_144548 --set last_change_max_start_s=6.0 2>&1 | grep "time per tick"
python3 practice/sim/slot_replay.py practice/sim/runs/20261001_150014 practice/sim/runs/20261001_144548 2>&1 | grep "time per tick"
```
Expected: 두 줄의 median/p95/max. 둘 다 보고서와 스펙 `### A-1 판단 시간` 에 적는다. p95 가 100 ms 를 넘으면 DONE_WITH_CONCERNS 로 보고한다(판단 주기 밀림 위험, HANDOFF §5.3).

- [ ] **Step 6: 커밋**

```bash
bash practice/check_submission.sh 2>&1 | grep -A6 "outside practice"     # 기존 3줄만
git add src/foscar_planning/config/highway.yaml src/foscar_planning/foscar_planning/highway_config.py src/foscar_planning/foscar_planning/highway_policy.py src/foscar_planning/test/test_highway_last_change.py docs/superpowers/specs/2026-10-01-highway-overtake-design.md
git commit -m "highway: the last change (3->2) may wait up to last_change_max_start_s to pass a slow lane-2 car (A-1)"
```

---

### Task 4: 시뮬 측정 (고속도로 구간 5회 + 풀랩 2회)

**Files:**
- Modify: `docs/superpowers/specs/2026-10-01-highway-overtake-design.md` (`### A-1 결과`)

- [ ] **Step 1: 빌드 확인**

`colcon build --symlink-install --packages-select foscar_planning` (yaml 이 share 에 반영되도록). `ps aux | grep -E "loop_runner|ros2 launch" | grep -v grep | wc -l` → 0.

- [ ] **Step 2: 고속도로 구간 5회**

```bash
source install/setup.bash && export PYTHONPATH=$PWD/.camera-deps:$PYTHONPATH
python3 practice/sim/loop_runner.py Practice3_CP08 --runs 5 --end-s 1745 --watch-from 1110 --stop-s 30 2>&1 | grep -E "results:|README"
python3 practice/sim/analysis/highway_summary.py practice/sim/runs/<stamp>
```

- [ ] **Step 3: 풀랩 2회**

```bash
python3 practice/sim/loop_runner.py Practice3_START --runs 2 --end-s 2165 --watch-from 0 --stop-s 60 --timeout 400 2>&1 | grep -E "results:|README"
python3 practice/sim/analysis/highway_summary.py practice/sim/runs/<stamp>
```

- [ ] **Step 4: 3→2 결정 확인**

`grep '"3->2"' practice/sim/pass_records.jsonl | grep -E '<stamp1>|<stamp2>'` 로 각 회차의 3→2 `change` 레코드(`plan.mode`, `plan.rear`, `plan.front`, `s`, `v_kmh`)를 뽑아, 2차로 차 앞으로 들어간(rear 가 비지 않은) 회차 수를 센다.

- [ ] **Step 5: 기록**

스펙 `### A-1 결과` 에: 회차별 결과(PASS/충돌), CP10→CP13·CP10→CP12, lane2 slow follow, 3→2 착지(앞/뒤), 풀랩 랩타임과 CP13 통과 속도(`python3 practice/sim/analysis/speed_limit_check.py` 사용법을 파일 머리에서 확인해 실행). 기준(`### 기준`, `### A-1 기준 지표`)과 나란히 비교하고, 충돌·접촉이 하나라도 있으면 그 회차의 s·상대를 적는다. 효과가 없거나 나빠졌으면 그렇게 쓴다.

- [ ] **Step 6: 커밋**

```bash
git add docs/superpowers/specs/2026-10-01-highway-overtake-design.md practice/sim/pass_records.jsonl
git commit -m "docs: A-1 sim results (5 x Practice3_CP08, 2 full laps)"
```
