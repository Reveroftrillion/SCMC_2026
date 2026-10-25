# 인수인계 — 장애물 회피(AVOID/YIELD) 로직 개선

## 후속 작업 현황 (2026-09-20)

- **P2 완료 + 인지 오탐 제거 (23:15). 실차 검증 통과.** 아래 6건을 고치고 30 km/h 로 박스·보행자 구간을
  통과시켰다: 이벤트 60건 **전부 실제 물체(오탐 0)**, 박스 회피·복귀 정상, 보행자 정지 후 출발, 충돌 0,
  조향 변화 중앙 **0.08°**(이전 0.33°). 단위 테스트 77개 통과.

  1. **가드레일·연석 오탐** — 판정 기준이 분류에 따라 달라(정지물 2 m / 이동물 5 m) 오분류가 곧 필터
     구멍이었다. 종류와 무관한 기하 판정 하나로 교체: 지도(`link_set.json`) 도로 안 + 경로 가로폭 ≤ 2 m
     + 통로(반폭 1.5 m) 침범. 장애물 미스폰 시나리오에서 발행 **189건 → 1건**. (foscar_lidar README 4-6.5)
  2. **추종선 떨림·계단식 복귀** — 플래너가 매 틱 0.25 m 격자에서 다시 골라 −1.75↔−1.50 을 오갔다.
     선회 시작 후 오프셋을 **줄이지 않는** 래치 추가 (키우는 것은 허용).
  3. **오프셋 적용을 시간 램프 → s 곡선으로** — `core.py offset_at(s)`. 요청이 바뀐 지점부터
     `L = max(transition_min_m, v·T)` 동안 5차 곡선. 시간 램프는 속도가 빠를수록 복귀 거리가 늘고
     (20 km/h 11 m, 40 km/h 22 m) 기준선이 차보다 먼저 가서 횡오차 FAULT 가 났다. 횡오차 판정도
     "차선 중심과 오프셋 선 **사이**" 밴드 기준으로 변경.
  4. **조향 과다** — 3차 smoothstep 은 양 끝 곡률이 최대라 전이 시작·끝에서 조향이 계단처럼 꺾인다
     (복귀에서 24°). **5차(6u⁵−15u⁴+10u³)** 로 교체(양 끝 곡률 0) + 복귀 시간 1.5 → 2.0 s 로 진입과 대칭화.
  5. **회피 시작이 이름** — `entry_buffer_m` 6 → 1.5. 시작 거리 = `front_m + entry_buffer_m + L(v)`
     (시간 딜레이가 아니라 거리 조건이며 매 틱 재계산, 한 번 참이면 latch).
  6. **유령 박스 / 보행자 회피** — 기억 삭제 조건을 "아직 앞에 있나" → **"볼 수 있나"**(`ctx.s + front_m
     > box.s_min`)로 바꿔 건너간 보행자를 잊게 했고, `PEDESTRIAN`/`VEHICLE` 로 분류된 자리는
     `dynamic_hold_s`(5 s) 동안 정적 장애물로 취급하지 않는다(사람 옆을 스쳐 지나가지 않음).
     ⚠ 이 판정에 **순간 속도를 쓰면 안 된다** — 정적 박스도 측면이 드러나며 중심점이 점프해 최대
     6.85 m/s 가 나오고(프레임의 27 %), 실제로 박스를 못 피하고 들이받았다.

  남은 것: 급제동 시 노면 오검출(지면 평면 피팅 필요), 처음부터 멈춰 있는 보행자, 측후방 ROI 확장.
  자세한 근거·수치는 `src/foscar_lidar/오탐분석.md` 8-3 절과 그 아래 "다음에 볼 것".

- **P2(b) 복귀 타이트 — 원인 확정·수정 (17:50).** 기억(`exit_pass_m 5`)은 남아 있어도 **플래너 `relevant()` 가
  `s_max_seen + behind_m(1)` 에서 박스를 버려 `d=0`** 을 냈다. 라이다는 앞면만 보므로(3 m 상자가 size_x ≈ 1.3)
  후륜축이 실측 s_max 를 1 m 지난 시점 = **뒤 범퍼가 실제 상자 뒤를 0.4 m 지난 시점**에 오프셋이 풀렸다.
  수정: (1) 이벤트 박스를 `s_max + hidden_length_m(2.5)` 로 늘려 실제 크기로 취급, (2) 복귀 기준을
  `clear_s = s_max + tail_m(2.0) + 뒤오버행(0.835)` 로 정의, (3) 플래너 `behind_m` 을 같은 값으로 맞춰
  기억 삭제와 `d=0` 시점을 일치시킴. `exit_pass_m` 삭제 → `hidden_length_m / tail_m / car_length_m`.
  통합 테스트(50 km/h): 복귀 요청 s=335.30 = 뒤 범퍼가 상자 뒤 **+3.97 m** (수정 전 +0.4 m). PASS, FAULT 0.
  단위 28 / 통합 PASS. **실차(10·30 km/h) 재검증 필요 — 특히 `tail_m` 2.0 이 충분한지 화면으로.**
  남은 P2: `core.py:307 offset_rate 0.5` 파라미터화·1.0 시험(팀 3 한 줄), 30 km/h 진입 재검증.

- **최신 스폰 (17:14 재확인): s=252.60 m, map (−73.10, −216.86), yaw=66.80°.
  경로 횡오차0.008 m, 방향 차이0.95°, 정차 Manual/P. 상자(s329)까지 약76.4 m 전.**
  아래17:10의 s396~398 스폰은 이전 위치다. 이번 확인에서는 제어 송신 없이 센서만 읽었으며
  제어 수신 연결 정상 여부는 아직 재검증하지 않았다. 기록: `~/foscar_logs/spawn_20260920_171415.json`.

- 사용자 관찰: 22:19 상자 시험의 **복귀는 양호**, 회피 시작이 너무 이름. 저속/고속 모두 안정적인 통과가 목표.
- 정지선 감속 중 큰 조향 요동 원인 확인/수정: `core.py`가 조향 주시점까지 `hold_s`로 잘라 정지 직전
  작은 횡오차를 증폭시켰음. 조향 주시점과 정지 속도 목표를 분리. 5 cm 오차/정지 0.5 m 전 회귀 시험은
  수정 전 34.9° 조향으로 실패, 수정 후 통과. 정지/재출발 기존 시험도 통과.
- 시뮬 정지 시험: 고정 stop_s=315, s≈265 출발. 최소 주시 거리 3 m에서 마지막 3 m 조향 −1.21~+3.09°,
  4 m에서는 −0.56~+1.84°. 최대 횡오차 각각 5.3/6.0 cm, 앞바퀴 정지 s≈315.04.
  `low_speed.yaml`의 최소 주시 거리를 4 m로 반영. **사용자 관찰상 작은 조향 보정은 아직 남아 있음.**
  로그: `~/foscar_logs/stop_fixed_20260919_222931/`, `stop_lookahead4_20260919_223416/`.
- 회피 진입 수정 전 속도 비교 (둘 다 주시 거리4 m): 횡오차0.8 m 첫 초과가 10 km/h에서는 s=304.13,
  30 km/h 상한 접근에서는 s=312.06. 저속에서 약8 m 일찍 옆으로 나감.
- 10 km/h 시험에서 상자(s≈328.5)가 잠깐 PEDESTRIAN으로 분류돼 YIELD가 회피 오프셋을0으로 덮어씀.
  s≈319.25 정지 → 기존 deadlock8초로 재출발 → s≈350 자동 종료. **분류 흔들림과 상태 간 횡명령 충돌은 미해결.**
  로그: `~/foscar_logs/avoid_low10_lh4_20260919_223624/`, `avoid_approach30_lh4_20260919_223835/`.
- 새 `avoid.py` 진입 조건: 인식/감속은 먼저 하되, 장애물 앞면까지 거리가
  `front_m + entry_buffer_m(10 m) + 현재 속도 × 남은 램프 시간` 이내일 때 선회 시작.
  정차 중 새 선회 시작 금지, 시작 후 감속해도 취소하지 않는 latch 추가. **복귀 로직은 변경하지 않음.**
  공간 경로 전체를 만드는 방식은 아니다. 수정 후10 km/h 시뮬 시험에서 선회 요청 s=304.45,
  횡오차0.8 m 첫 초과 s=312.06 (수정 전304.13 대비 약7.9 m 늦춤). AVOID 해제s=335.38.
  상자 구간 이벤트111개 모두 OBSTACLE, YIELD/FAULT 없이 s=350 종료. 수정본의30 km/h 재검증은 남음.
  실제 충돌/차량 전체 최소 간격은 bag만으로 확정할 수 없음. 사용자 화면 관찰을 함께 확인할 것.
  로그: `~/foscar_logs/avoid_gated_low10_start_20260920_170030/`.
- 제어26개/판단27개 테스트 통과. 회피 진입 수정본은 주시거리3 m와 최종4 m 설정에서 통합 시험 통과.
  최종4 m 통합 결과: 252 targets, AVOID/CRUISE, 최대 오프셋2.5 m, FAULT 없음.
- 사용자가 시나리오 처음(s≈0.5)으로 재배치하여 수정본10 km/h 시험을 수행함. 현재 s≈350에서 종료.
  s≈184~195에서도 AVOID에 잠깐 진입했지만 오프셋0을 유지했음 (상자 이전의 인식 오진입 여부는 추가 확인).
- **사용자가 이후 스폰을 변경함 (2026-09-20 17:10 확인): s≈396~398, map x≈−60, y≈−75,
  진행방향 약90°.** 기존 상자(s329)를 지난 뒤, 보행자(s460) 앞이다. global 경로 오차 약4~8 cm,
  route_offset=0. 매 시험 전 실제 수신 위치를 확인하고 기존 s265 가정을 쓰지 말 것.
  스폰 관측: `~/foscar_logs/spawn_20260920_1710.json`.
  새 스폰20km/h 시험은 Auto/D 전환 실패로 출발하지 못함(센서 정상, 제어 라우팅 .10→.11 정상).
  사용자에게 F4 Load/제어 수신 연결 확인 요청 중. 로그: `~/foscar_logs/avoid_new_spawn_ped20_20260920_171147/`.
- 아래 원래 계획의 도로 마스크·좌우 경계 선택·기억 정책 변경은 아직 미구현. 새 관찰을 우선해 진행할 것.

---

> 작성 2026-09-19 22:10. 대상: 이 작업을 이어받는 에이전트/팀원. 이 문서만 읽고 바로 착수할 수 있게 썼다.
> 레포 `~/2026Morai/2026foscar` (ROS 2 Humble, Python). 
---

## 0. 한 줄 요약

MORAI K-City 자율주행 대회(2026-10-30) 코드. 오늘 `foscar_planning`(FSM) 패키지와 대회용 통합 런치를 새로 만들어 **실제 시뮬에서 상자 회피는 성공**했지만, 실주행에서 아래 **4가지 문제**가 관찰됐다. 이 문서의 §4 계획대로 고치고 §5 기준으로 검증하면 된다.

---

## 1. 배경 (3분)

- 미션: Ioniq5 로 2,184.6 m 한 바퀴. 패널티 서열 **차선접촉(3 s 지속마다 5 s) < 충돌(15 s) < 경로이탈(≈24 s)**.
- 장애물 관련 미션 위치 (s = 전역경로 진행거리 m): **329 정적 상자(2×3×2 m)**, **460 보행자 횡단(10 km/h)**, **890 회전교차로 NPC(20 km/h, 5 s 주기)**, 터널(1881~1998) 안 랜덤 미션.
- 신호등(카메라팀)은 **의도적으로 미통합**. `states` 목록에 `stop_line` 이 없어서 신호 때문에 서는 일이 없다. 붙이는 자리는 `README.md §9`.
- 규약 문서: `~/2026Morai/개발계획서.md` §1~2 (좌표·토픽·메시지), `~/2026Morai/Readme, 각종정보파일/PROJECT_MASTER.md` (미션·채점).

## 2. 지금 구조 (오늘 만든 것 포함)

```
competition.launch.py  (foscar_bringup)      ← bash run_competition.sh [rviz:=true] [drive:=false]
├ foscar_control   sensor_bridge · localization(GPS+IMU, 터널 추측항법) · path_follower(tracker, require_target=true)
├ foscar_lidar     lidar_bridge · obstacle_node  →  /events (OBSTACLE / PEDESTRIAN / VEHICLE, base_link x,y + s)
└ foscar_planning  behavior_node  ← /events, /localization/{odometry,s,mode}, /map/route  →  /behavior/target (유일 발행자)
      state.py           코어. 걸린 상태들의 Target 을 합침: lat_offset=최상위, stop_s=min, v_max=min
      frenet.py          경로↔(s,d) 변환 (d 좌 +), base_link→map, 폐루프 안전 s_offset
      avoid_planner.py   후보 d=-3…+3 (0.25 간격) × 복도 충돌검사 × 비용 → d, v_max   ★ 손댈 곳
      states/cruise.py   (0)  구간 속도: 도심 50 / 430~470·860~900 → 20 / 1119~1878 → 60 km/h
      states/avoid.py    (70) 정적 장애물 기억·회피·막히면 정지                            ★ 손댈 곳
      states/yield_.py   (80) 보행자·NPC 앞 정지, 8 s 데드락 해제 (파일명 _ : yield 예약어)  ★ 손댈 곳
      states/tunnel.py   (40) v_max 22 km/h
      behavior_node.py   Event→EventInfo(map x,y / global s / d) 변환, 20 Hz, /behavior/markers  ★ 손댈 곳(도로 마스크)
      bev_node.py        RViz BEV 배경 (차선=link_set 중심선±1.75, s 눈금, 자차 박스)
      config/behavior.yaml   모든 파라미터 ("<상태>.<키>"). 모르는 키 적으면 노드가 죽음(의도)
tools/fake_event.py      가짜 이벤트 (obstacle 329 / obstacle 329 size 1 7 / pedestrian 460 d 4 vy -1.4)
```

tracker 가 `DrivingTarget` 을 어떻게 쓰나 (`foscar_control/core.py` `Controller.step`, "mission hook" 주석):
- `lat_offset` → **0.5 m/s 로 램프**(`self.offset_rate`, core.py:307 하드코딩) 해서 경로를 옆으로 민 점을 pure-pursuit 로 추종. ±3 m 클립. 횡오차 FAULT(1.5 m)는 오프셋을 뺀 값 기준.
- `stop_s` → 앞바퀴(후륜축+3.0 m) 기준, **1.0 m/s² 램프**로 정지.
- `v_max` → 곡률 속도프로파일에 min. `v_max=0` 이면 속도오차로 브레이크 최대 0.5(≈5 m/s²) — YIELD 가 급제동에 쓴다.
- `/behavior/target` 이 0.5 s 안 오면 tracker 가 세운다 (`require_target`).

## 3. 실주행 결과 (2026-09-19, 시뮬 PC 192.168.0.11)

| 회차 | 결과 |
|---|---|
| 1차 (AVOID 만) | s=329 상자: 상자가 차선 왼쪽에 치우침(`y=+2.26`) → **오른쪽 −1.88 m 로 비켜 통과, 충돌 0**. s=460 보행자: 48 km/h 로 접근, **11 m 앞에서 첫 감지 → 0.6 s 뒤 충돌**. bag: `~/foscar_logs/bags/run_20260919_214319` |
| 2차 (YIELD + 구간감속 추가 후, 사용자가 직접) | 아래 4가지 문제 관찰. bag 없음(`bag:=false`). tracker CSV: `~/foscar_logs/follow_20260919_2205*.csv`, `follow_20260919_2206*.csv` |

### 관찰된 문제 4가지와 원인 진단

**P1. 상자를 보고 중앙선(노란선)을 넘어 회피했다.**
원인: `avoid_planner.py` 가 좌/우를 비용만으로 고르고, 동점이면 `w_left_bias` 로 왼쪽. 노란 중앙선인지 흰 실선인지 모른다.
답은 맵에 있다: `link_set.json` 링크마다 `left_lane_change_dst_link_idx` / `right_lane_change_dst_link_idx` / `can_move_*_lane` (같은 방향 옆 차선 유무), `opp_traffic`. 왼쪽에 같은 방향 차선이 없으면 그 경계는 중앙선.

**P2. 너무 일찍 꺾기 시작하고, 복귀는 장애물을 스치듯 타이트하다.**
원인 (a) 진입(40 m 전)과 동시에 `lat_offset` 전부를 요청 → 램프가 즉시 시작. (b) 해제 조건 `ego_s > s_max + exit_pass_m(5)` 에서 `s_max` 가 **라이다가 본 앞면 기준**(실측 size_x 1.3 m, 실제 3 m). (c) 오프셋 목표가 0 이 되는 순간 pure-pursuit 의 lookahead 점(3~10 m 앞)이 안쪽으로 가서 **차 뒤가 빠지기 전에** 파고든다. 근본: "장애물 기준 어디서 시작·유지·복귀" 가 없고 시간 램프에 맡겨져 있다.

**P3. 가드레일 등 도로 밖 구조물이 장애물로 잡힌다.**
원인: `obstacle_node` 의 경로 필터는 "경로 중심선에서 횡 `static_max_offset_m`(2.0) 이내" 뿐. 곡선·갓길에서 가드레일이 들어온다.
권장 해법: **맵 도로 마스크** — 634개 링크 전부의 `중심선 ± width/2(=1.75)` 를 도로 영역으로 두고, 어느 링크 위에도 없는 덩어리는 버린다 (맵은 경로와 오차 0). 반사율(intensity)로 차선을 찾는 아이디어는 점선·교차로에서 끊기므로 이걸로 부족할 때 2차.

**P4. 오검출 장애물이 사라진 뒤에도 너무 오래 기다린다.**
원인: `behavior_node` 이벤트 만료 0.5 s + `avoid.py` 기억 `memory_s` 3 s (라이다 ROI 가 전방 x≥0 뿐이라 옆에 온 장애물이 이벤트에서 사라지는 걸 버티려고 넣음). 막힌(blocked) 오검출이면 그동안 `stop_s` 로 서 있는다. `obstacle_node` 는 미스 중 재발행은 안 한다(`misses==0` 일 때만 발행, obstacle_node.py:651). P3 를 고치면 대부분 사라지지만 기억 정책도 고쳐야 한다.

## 4. 작업 계획 (이 순서로)

### 4-0. 데이터 확인 (10분)
- 1차 bag 으로 이벤트 분포 보기 (스크립트 예시는 이 세션에서 썼던 것: `rosbag2_py` 로 `/events`, `/behavior/target`, `/localization/s` 읽기 — `README.md` 없음, 직접 10줄 쓰면 됨).
- 2차는 bag 이 없으니 `follow_2205*/2206*.csv` (20 Hz: state, x, y, s, cross_track, 명령) 로 회피 시작 s·복귀 s 만 확인.
- 앞으로 실주행은 **항상 `bag` 기본값(true)** 으로.

### 4-1. 도로 마스크 + 링크 메타 (P3 → P4 대부분)
- 새 파일 `foscar_planning/foscar_planning/road.py` (ROS 없음):
  - `RoadMap(link_set_path)`: 전 링크 점을 하나의 배열로 + `scipy.spatial.cKDTree` (없으면 numpy brute force, 5.7만 점이라 가능). 링크 좌표는 이미 map 기준(원점 302595/4124145 빼진 값), 폭은 전부 3.5.
  - `is_on_road(x, y, margin=0.3) -> bool`: 최근접 링크 점 거리 ≤ width/2 + margin.
  - `link_at(x, y)` → 링크 dict. `neighbors(link)` → `(has_left_same_dir, has_right_same_dir, opp_traffic)`.
  - 기본 경로: `package://foscar_bringup/maps/link_set.json` (bev_node.py 의 `DEFAULT_LINK_SET` 재사용). 없으면 마스크 끄고 경고.
- `behavior_node.on_event`: map 좌표로 바꾼 뒤 `is_on_road` 실패면 **버림** (카운터 로그 5 s 마다). 파라미터 `road_mask: true`, `road_margin_m: 0.3`.
- 테스트: 인공 링크(직선 2개)로 on/off 판정, 경계 ±margin.

### 4-2. 맵 기반 좌우 선택 (P1)
- `AvoidPlanner.plan(..., side_info)` 에 `(left_ok, right_ok)` 전달. 후보 d 가 `+lane_half` 를 넘는데 `left_ok=False` 면 비용 `+w_center(5.0)`(중앙선), `-lane_half` 를 넘는데 `right_ok=False` 면 `+w_shoulder(1.5)`(갓길). `w_left_bias` 삭제.
- `avoid.py` 는 `RoadMap.link_at(ctx.x, ctx.y)` 로 현재 링크의 이웃 정보를 얻어 넘긴다 (RoadMap 은 behavior_node 가 만들어 `ctx` 에 넣어 주는 게 깔끔: `Context.road`).
- 테스트: 상자 중앙 + 왼쪽 차선 없음 → 오른쪽; 왼쪽 같은 방향 차선 있음 → 왼쪽; 둘 다 없음 → 갓길(오른쪽).

### 4-3. s 기반 회피 프로파일 (P2)
- `avoid.py` 에 장애물 기준 구간 정의: `lead_m`(8) 앞에서 오프셋 **완성**, `s_max + hidden_length_m`(2.5: 라이다는 앞면만 봄) 까지 유지, **차 뒤가 빠진 뒤** 복귀(`s_max + hidden + car_length 4.635 + tail_m 2`).
- 시작 시점: 램프 시간 `|d|/offset_rate` 를 역산해, `(s_min − lead − ego_s)/v ≤ |d|/offset_rate` 가 될 때부터 `lat_offset=d` 요청. 그 전에는 0 (AVOID 는 걸려 있되 속도만 관리). 지금 있는 `approach_speed()` 는 이 계산과 합칠 것.
- 복귀: 위 복귀 s 전까지는 `lat_offset=d` 유지, 그 뒤 0. `exit_pass_m` 은 복귀 s 기준으로 재정의.
- 램프 속도: `core.py:307 self.offset_rate = 0.5` 를 파라미터(`offset_rate_mps`, low_speed.yaml)로 빼고 **1.0** 제안. 팀 3 코드이니 한 줄 변경임을 알리고 진행. `avoid.offset_rate_mps` 는 반드시 같은 값.
- 테스트: 40 m 전 진입 시 lat_offset 이 처음엔 0, 램프 역산 시점에 d 로 바뀜; 복귀는 s_max+hidden+4.6+2 이후에만.

### 4-4. 빨리 잊기 (P4)
- `avoid.py observe()`: 기억 유지는 장애물이 **차 앞 `front_m`(3.8) 안쪽 또는 옆/뒤** 일 때만. 아직 그 밖(=ROI 안)에 있는데 `memory_ahead_s`(0.5) 동안 안 보이면 삭제. `memory_s` 3 → 이 역할로 축소.
- `yield_.py`: `gone_s` 1.5 → 1.0.
- 테스트: 앞 20 m 오검출이 사라지면 0.5 s 안에 AVOID 해제 / blocked 정지 해제.

### 4-5. 검증 (§5) 후 문서: `foscar_planning/README.md` 의 AVOID/YIELD 절과 `behavior.yaml` 주석 갱신.

## 5. 검증 방법 / 완료 기준

```bash
cd ~/2026Morai/2026foscar && source /opt/ros/humble/setup.bash && source install/setup.bash
colcon build --packages-select foscar_planning foscar_bringup foscar_control        # 고친 패키지만
(cd src/foscar_planning && python3 -m pytest test/ -q)                            # 24개 + 새 테스트, 전부 통과
(cd src/foscar_control && python3 -m pytest test/ -q)                             # 25개
python3 src/foscar_planning/test/integration_competition.py                       # 런치 통째로, 루프백 가짜 차량 ~40 s
```
- `integration_competition.py` 는 s=285 출발 → `fake_event obstacle 329` → s=348 까지. 기존 PASS 유지 + 필요하면 시나리오 추가(오른쪽 회피 기대로 바뀜).
- 실주행 완료 기준 (시뮬 PC 준비 후 `bash run_competition.sh rviz:=true`):
  - [ ] 상자 중앙 배치: **오른쪽**으로 비킴, 충돌 0, 차선 밟은 시간 < 3 s, 복귀는 차 뒤가 상자를 완전히 지난 뒤
  - [ ] 가드레일 구간(1차 bag 에서 오검출 났던 s) 주행 중 AVOID/YIELD 오진입 0
  - [ ] 오검출을 강제로 만들어(`fake_event obstacle <ego_s+20>` 를 켰다 끔) 끈 뒤 1 s 안에 CRUISE 복귀
  - [ ] s=460 보행자: 20 km/h 로 접근, 정지, 지나가면 출발, 충돌 0

## 6. 함정 (오늘 실제로 밟은 것)

- **`pkill -f competition`** 은 자기 셸까지 죽인다. `pkill -INT -f "ros2 launch foscar_bringup competition"` 처럼 좁게.
- launch 의 `IncludeLaunchDescription(launch_arguments=...)` 는 **바깥 스코프까지 덮어쓴다**. `GroupAction(scoped=True, forwarding=True)` 로 감쌀 것 (이미 고침, `competition.launch.py` 주석).
- `Marker.points += [...]` 는 O(n²) — 리스트 만들어 한 번에 대입 (bev_node 주석).
- `quaternion_yaw(w, x, y, z)` 인자 4개.
- `Route.project(x, y, previous)` 의 탐색창은 previous −3~+8 m 뿐 → 장애물 투영엔 `frenet.to_frenet` (뒤 30/앞 100 m) 사용. 폐루프라 출발점에서 s 가 한 바퀴 틀릴 수 있음 → `frenet.s_offset_from`.
- 시뮬 PC 는 항상 GPS/IMU 를 쏘고 있다. `drive:=false` 는 안전하지만 **`drive:=true` 는 즉시 차가 출발**한다. 시뮬 쪽에서 시나리오 로드 후 `F4 > Load` 안 하면 제어 패킷을 안 받아 `Auto/D handshake timeout` (이 PC 의 시뮬 쪽 IP 는 `192.168.0.10`).
- 로컬에서 ROS 만 띄워 테스트할 땐 `ROS_DOMAIN_ID=7x ROS_LOCALHOST_ONLY=1` 로 격리 (integration 테스트가 그렇게 함).
- 시간은 `ctx.now` 만. `time.time()` 금지 (bag 재생 깨짐). 주석·로그는 한국어, 기존 파일 톤 따를 것.
- 상자 회피에 필요한 오프셋은 **2.45 m** (상자 반폭 1 + pad 0.2 + 차 반폭 0.946 + 마진 0.3). 마진을 올리면 ±3 m 안에서 못 지나가 **선다**. `avoid_planner.py` 머리 주석 참조.

## 7. 파라미터 현재값 (behavior.yaml)

| 키 | 값 | 비고 |
|---|---|---|
| `avoid.enter_distance_m` | 40 | 라이다 ROI 40 m |
| `avoid.margin_m` / `pad_m` / `d_max_m` | 0.3 / 0.2 / 3.0 | 위 2.45 m 계산 |
| `avoid.v_avoid_mps` / `v_avoid_max_mps` / `line_time_budget_s` | 5 / 8 / 2.5 | 차선 밟는 시간 < 3 s |
| `avoid.offset_rate_mps` | 0.5 | tracker `core.py:307` 과 같아야 함 |
| `avoid.memory_s` | 3.0 | P4 에서 축소 예정 |
| `yield.enter_distance_m` / `cross_d_m` / `predict_s` / `stop_gap_m` / `deadlock_s` | 25 / 2.5 / 2 / 6 / 8 | |
| `cruise.zone_*` | 430~470→20, 860~900→20, 1119~1878→60 km/h, 그 외 50 | 보행자 감지 ~11 m 실측 |
| `obstacle_node static_max_offset_m` (foscar_lidar) | 2.0 | P3 후 그대로 두되 마스크가 앞단에서 거름 |
