# foscar_planning — FSM 코어 (`/events` → `/behavior/target`)

개발계획서 §2 의 "판단" 자리. `behavior_node` 가 `/behavior/target` 의 **유일한 발행자**이고,
tracker(`path_follower`) 는 이것 말고 아무것도 안 본다. 어떤 상태를 돌릴지는 `states` 파라미터가 정한다.

```
foscar_planning/
  state.py            BehaviorState / Context / Target / Behavior(코어: 걸린 상태 합치기)   ← ROS 없음
  frenet.py           경로 ↔ (s, d) 변환, base_link → map, s_offset                       ← ROS 없음
  avoid_planner.py    "옆으로 몇 m" 를 고르는 Frenet-lite 플래너                            ← ROS 없음
  states/
    cruise.py   (0)   구간별 속도 상한. 항상 걸려 있음                                  팀 3
    tunnel.py   (40)  s=1875~2000 또는 DR; 추가 속도 상한 기본 해제                            팀 3
    avoid.py    (70)  정적 장애물 회피 (AvoidPlanner 사용)                              팀 2
    yield_.py   (80)  보행자·NPC 앞 정지 (파일명에 _ : yield 는 예약어. states 엔 'yield')   팀 2
    stop_line.py(60)  [카메라팀이 추가] 신호 정지선. README §9 참조                     팀 1
  behavior_node.py    ROS 노드. Event → EventInfo(map, s, d) 변환, 20 Hz 틱, 마커
  bev_node.py         RViz BEV 배경: 차선(link_set), s 눈금, 자차 박스
config/behavior.yaml  모든 파라미터 (상태별 "<이름>.<키>")
rviz/bev.rviz         BEV 화면 구성
test/test_planning.py         순수 로직 테스트         cd src/foscar_planning && python3 -m pytest test/ -q
test/integration_competition.py  competition.launch.py 를 루프백 UDP 로 실제 실행 (~40 s, 시뮬 불필요)
```

## 코어 규칙 (state.py)

매 틱 모든 상태에 대해: 안 걸려 있으면 `check_entry`, 걸려 있으면 `check_exit`. 걸린 상태 전부의
`compute_target` 을 합친다:

| 필드 | 합치는 법 |
|---|---|
| `state`, `lat_offset` | priority 가장 높은 상태 것 |
| `stop_s` | 걸린 상태들 중 **가장 가까운 것** (min) |
| `v_max` | 걸린 상태들 중 **가장 낮은 것** (min) |

그래서 빨간불(`stop_s`) + 장애물(`lat_offset`) 이 14 m 차이로 겹쳐도 둘 다 살아남는다.
상태는 자기 일만 내면 된다.

## 상태 하나 추가하기

`states/<이름>.py` 에 `BehaviorState` 서브클래스 **하나**:

```python
from ..state import BehaviorState, Context, Target

class StopLine(BehaviorState):
    name = 'STOP_LINE'
    priority = 60
    defaults = {'enter_distance_m': 40.}     # → ROS 파라미터 stop_line.enter_distance_m

    def check_entry(self, ctx: Context) -> bool: ...
    def check_exit(self, ctx: Context) -> bool: ...
    def compute_target(self, ctx: Context) -> Target:
        return Target(state=self.name, stop_s=...)
```

- `ctx` 에 `s x y yaw v now events loc_mode route s_offset`. 이벤트는 `ctx.events_of('TRAFFIC_LIGHT')`.
  각 `EventInfo` 는 map 좌표 `x y`, global `s`, 경로 기준 횡거리 `d`(좌 +), 크기, 속도, `value`.
- 시간은 `ctx.now` 만. `time.time()` 금지 (bag 재생이 안 됨).
- `defaults` 에 없는 키를 yaml 에 적으면 노드가 시작하면서 죽는다 (오타 방지).
- 켜기: `behavior.yaml` 의 `states` 또는 런치 인자 `states:=cruise,avoid,tunnel,stop_line`.
- 테스트는 `test/test_planning.py` 처럼 `Context` 를 직접 만들어 돌린다. 시뮬 없이 된다.

## AVOID 가 하는 일 (avoid.py + avoid_planner.py)

1. `OBSTACLE` 이벤트 중 경로 위(|d| ≤ 1.5 m)인 것을 **기억**한다 (라이다 ROI 가 전방뿐이라 옆에 오면 이벤트가 사라진다).
2. 후보 d = −3 … +3 m (0.25 간격, 25개). 차폭+마진 복도가 장애물 Frenet 박스와 겹치면 탈락.
3. 비용 = |d| + 차선 밖 폭×2 + 반대쪽으로 뒤집으면 +3 + 변화량×0.5 (왼쪽 살짝 우대) → 최소.
4. 회피 전용 `v_max`/선제 감속/차선 접촉 시간 기반 증속 계산은 없다. 속도는 CRUISE·정지 요청과
   tracker의 기존 경로 속도 프로파일이 담당한다. 장애물 인식과 선회 시작은 분리한다:
   진입 곡선 시작은 장애물 앞면에서 `front_m + entry_buffer_m + max(transition_min_m, v × transition_time_s)` 앞이다.
   최대 주시거리(10m) 전에 `entry_s`·`entry_length_m`를 전달해 주시점이 곡선에 연속적으로 들어가게 한다.
   계획 요청 시점과 실제 차체의 횡이동 시작은 다르며, 곡선 길이는 요청 시점 속도로 고정한다.
5. 곡선은 tracker가 s에 대한 5차 다항식으로 만든다. 도중 목표 오프셋이 커져도 위치·접선·곡률을 이어주며,
   같은 방향 보정은 가능한 한 기존 완료 위치를 유지한다. 전이 중 횡오차 적분은 고정해 추종 지연을
   조향 편향으로 누적하지 않는다. 의도한 횡기동 중 yaw 차이를 이탈로 보고 감속하던 경험적 보정은 적용하지 않는다.
   속도 목표의 상승은 기존 `accel_mps2`로 제한하며 정지·감속은 즉시 반영한다.
6. 최초 앞면에 `hidden_length_m`를 적용하며 측면이 보일 때 숨은 길이를 다시 붙이지 않는다.
   실측 뒤끝이 더 멀면 기억을 확장한다. 복귀 기준은 상자 뒤끝 + 뒤 오버행 + `tail_m`이다.
   지도상 거의 직선인 구간에서는 회전 차체와 장애물의 분리축 충돌 검사(0.2m 간격, 차폭에 margin 포함)를
   통과할 때만 복귀 기준을 `return_lead_m`(2m) 앞당긴다. 이 검사는 명목 경로의 기하 검증이며 실제 추종오차는
   실주행으로 별도 검증한다. 급곡선에서는 앞당기지 않는다.
   `return_s`(global s), `return_length_m`를 진입부터 전달해 PP가 완만한 복귀 곡선을 미리 볼 수 있게 했다.
   복귀 길이는 `max(return_min_m, transition_min_m, 진입 속도 × return_time_s)`이며,
   저속 출발 뒤 가속할 때도 곡선이 너무 짧아지지 않도록 `return_min_m=12m`를 적용한다.
   따라서 `lat_offset`은 복귀 중에도 **회피 최대 오프셋**을 유지하고, 실제 횡이동은 공간 곡선을 따른다.
   AVOID는 복귀 곡선 끝에서 해제된다. 회피 계획 전달 후 점군 분류 변경·미검출로 장애물 기억이 사라져도 곡선을 유지하고, 정지 요청에도 진입·복귀 곡선 정보를 함께 전달한다. 미션 경계에서는 기존 FSM 초기화를 따른다. 실제 기준선은 `/control/path`로 확인한다.
7. 지나갈 후보가 없으면 장애물 앞 `stop_gap_m`에 `stop_s`를 요청한다.

메시지 필드가 추가되어 `foscar_interfaces`와 제어·판단 패키지를 함께 빌드해야 한다.
`entry_length_m=0`, `return_length_m=0`인 기존 방식의 발행 코드는 재빌드 후 종전처럼 동작한다.

실주행 1회 시험은 `tools/run_avoid_test.sh`로 실행하면 종료/FAULT/시간 제한 뒤 주행 프로세스를 정리한다.
`tools/analyze_avoid.py <follow.csv> --plot <result.png>`는 지도 경로와 yaw로 차체 네 모서리를 계산한다.
표시된 이탈 시간은 경로 중심 ±1.75m(지도 차로 폭 3.5m) 근사 기준이며 실제 차선 페인트 판정과는 다를 수 있다.
충돌 모니터는 CollisionData 수신 패킷 수도 기록한다. 미수신은 무충돌로 검증된 것이 아니다.

숫자 감각: 차선 3.5 / 차폭 1.892 / 상자 2 m 중앙 → 필요한 |d| = 1.0+0.2+0.946+0.3 = **2.45 m**. 차가 통째로 옆 차선에
들어간다. 마진을 올리면 못 지나가서 선다. 값을 바꾸면 `avoid_planner.py` 머리의 계산부터 다시.

진입 시점 보정 시뮬 검증(2026-09-20): 10 km/h, 최소 주시 거리4 m에서 횡오차0.8 m 첫 초과 위치가
s=304.13 → 312.06으로 약7.9 m 늦어졌다. 수정 후 시험의 선회 요청은s=304.45, 해제는s=335.38.
상자 구간 YIELD/FAULT는 없었으며, 수정본의 다른 접근 속도 검증은 남아 있다.
기록: `~/foscar_logs/avoid_gated_low10_start_20260920_170030/`.

## YIELD 가 하는 일 (yield_.py)

1. `PEDESTRIAN`/`VEHICLE` 이 앞 25 m 안에서 경로 위(|d| ≤ 2.5)거나 2 초 뒤 경로를 지나갈 것 같으면 진입.
2. `stop_s = 대상 s − 6`. tracker 램프(1 m/s²)로 못 서는 거리면 `v_max = 0` (급제동, 브레이크 0.5 ≈ 5 m/s²).
3. 대상이 경로 밖으로 멀어지는 게 1 초 이어지거나 1.5 초 안 보이면 출발. 서서 8 초 기다리면 그냥 간다 (4 초 재진입 금지).
4. 걸린 뒤에는 같은 자리의 `OBSTACLE` 도 대상으로 본다 (보행자가 느려지면 종류가 바뀐다).

실측 2026-09-19: 48 km/h 로 s=460 보행자에 접근 → 11 m 앞에서 첫 감지 → 0.6 s 뒤 충돌. **어떤 상태도 이걸 못 막는다.**
그래서 `cruise` 구간표에 430~470 m 와 860~900 m 를 20 km/h 로 넣었다. 20 km/h 면 4 m 안에 선다.

## BEV 로 보기

```bash
bash run_competition.sh rviz:=true          # bev_node + rviz(bev.rviz) 같이 뜸
```

| 화면 | 출처 |
|---|---|
| 회색 중심선 + 흰/노란 경계선 (링크 폭 ±1.75 m 근사) | `bev_node` ← `link_set.json` (`-p link_set:=경로`) |
| 파란 자차 박스 + 후륜축/앞바퀴축 선, s 눈금(10 m) | `bev_node` |
| 라이다 점 (밝기 = intensity, 차선 페인트가 하얗게) | RViz PointCloud2 |
| 주황 박스 = 라이다 장애물 | `obstacle_node` `/perception/obstacle_markers` |
| 회색/빨강 얇은 선 = 후보 오프셋(가능/충돌), **초록 굵은 선 = 지금 시킨 경로**, 빨간 가로선 = stop_s, 차 위 글자 = 상태판 | `behavior_node` `/behavior/markers` |

뷰는 TopDownOrtho, Target Frame = base_link (차를 따라감). 화면 방향이 어색하면 Views 패널의 Angle 을 돌린다.

## 시뮬 없이 확인하는 법

```bash
# 1) 순수 로직
cd src/foscar_planning && python3 -m pytest test/ -q
# 2) 런치 통째로 (루프백 가짜 차량, s=285 출발 → s=329 상자 회피 → s=348)
source install/setup.bash && python3 src/foscar_planning/test/integration_competition.py
# 3) 실제 스택에 가짜 이벤트 넣기 (path_follower·localization 이 떠 있어야 함)
python3 tools/fake_event.py obstacle 329            # 상자
python3 tools/fake_event.py obstacle 329 size 1 7   # 도로 전체를 막는 벽 → stop_s 가 나와야 함
```

## 회전교차로

현재 동작, RViz 표시, 설정과 검증 범위는 [회전교차로 최신 개발 상태](../../ROUNDABOUT.md)에 정리합니다.

## 체크포인트 상위 미션 FSM (2026-09-25)

`competition.launch.py`는 `mission.enabled: true`로 실행하며 `config/missions.yaml`을 읽습니다.
일반 주행 / 상자 / 보행자 / 회전교차로 / 고속도로 / 터널마다 허용할 하위 상태와 속도 정책을 정합니다.
구간은 **후륜축 global s 기준 `[start_s, end_s)`**이며 준비·복귀 여유까지 포함합니다.

| 미션 | s 구간(m) | 허용하는 주요 기능 | 미션 상한 |
|---|---|---|---|
| 상자 | 275–390 | 정적 회피 | 60 km/h |
| 보행자 | 405–495 | 보행자 양보 (VEHICLE 분류는 무시) | 준비20 → 양보·통과 확인 후60 km/h |
| 회전교차로 | 860–920 | 진입 판단·전방 추종, 근거리 차량 보호 | 20 km/h |
| 고속도로 | CP10–CP13 (약1118.74–1741.72) | 기본 control: 순차 변경·CP12 복귀·NPC 추종 | 미션 고정 상한 없음, 곡률·앞차·변경 조건에 따른 제한 적용 |
| 터널 | 1875–2000 | 지도·측정 벽 기반 좌우 회피 | 미션·DR 고정 상한 없음 |

미션마다 쓰는 상태만 허용합니다. NPC 차량은 회전교차로·고속도로에만, 보행자는 보행자 미션에만 나오므로 차량 YIELD는 회전교차로에만 있습니다(고속도로는 전용 런타임).
| 나머지 | 위 구간 외 | 일반 주행 | 60 km/h |

`run_competition.sh`의 일반 구간 기본 상한은60 km/h입니다. 고속도로는 동일한 `missions.yaml` 체크포인트를 제어기도 읽어 고정 상한을 해제합니다. 터널은 미션·DR 추가 상한만 없고 일반 구간 상한을 따릅니다. 경로 곡률·장애물 감속과 고속도로 종료 전 감속은 유지합니다.

상자 구간은 기록된 s≈329 상자를 둘러싼 임시 여유 구간입니다. 추가 상자나 체크포인트가 있으면 표를 수정해야 합니다.
보행자는 기존 감지 여유를 포함한 405–495에서 활성화합니다. 검출 전에는20 km/h로 준비하고,
**실제로 보행자에게 양보한 뒤 자차 경로 통과가 확인되면 같은 구간 안에서도60 km/h 일반 상한으로 복귀**합니다.
`after_pedestrian_yield_kmh`로 조절하며, 단순 관측 소실이나 차량 양보만으로는 해제하지 않습니다.
해제 후에도 보행자 검출·정지는 활성 상태입니다. 새 보행자/신호/차량이 요청하는 정지는 우선합니다.
구간 전환·리스폰·시간 역행 시 완료 기억이 초기화되어 다음 시험은 다시20 km/h 준비 상태로 시작합니다.
이는 미션 상한 해제이며 제어기의 실제 가속률·곡률 제한을 바꾸지는 않습니다.
고속도로는 톨게이트 CP13에서 끝나며, CP12 경로 복귀와 별도입니다. CP13에서 터널 준비 시작1875m까지는 일반 구간입니다. CP 기반 경계는 공통 `load_mission_config`로 원본 경로에 투영합니다.
`speed_kmh: null`은 미션 상한만 없앱니다. 실행 인자 `speed_kmh`(기본60), 곡률·가감속 프로파일,
센서 장애 정지는 유지하며 DR 추가 제한은 기본 해제입니다. 고속도로 실주행을 더 높은 속도로 검증한 뜻은 아닙니다.

미션 전환은 하위 상태의 `check_exit()`를 기다리지 않습니다. 이전 정지 목표, 대기 타이머,
회피 오프셋 계획, 회전교차로 진입 허가·앞차 기억을 폐기하고 새 미션 상태를 구성합니다.
허용 목록에 공통으로 있는 YIELD도 새로 구성하므로 지난 미션의 대기를 이어받지 않습니다.
YIELD를 허용하는 구간에서만 새 차량 위협을 같은 tick에 다시 평가하며, 장애물이 실제로 있을 때 시간 초과만으로 강제 출발하지 않습니다.
전방 회피 경로의 실제 복귀는 기존 추종기의 곡선 전이로 처리합니다.

- 소폭 후퇴/경계 흔들림은 진행거리 최고값으로 처리해 지난 미션을 재진입하지 않습니다.
- 최고 진행거리보다 5m 초과 후퇴, 속도로 설명되지 않는 전방 위치 점프, 시간 역행, 경로 교체는 새 실행으로 초기화합니다.
- 미션 중간에서 실행해도 해당 미션을 즉시 선택합니다. 한 미션을 통째로 건너뛰어 양쪽 모두 CRUISE여도 상태를 정리합니다.
- 리스폰 시 센서/예측 기억도 초기화하고 이전·미래 시각의 라이다 패킷을 거부합니다. 정상 미션 전환에서는 관측을 유지합니다.
- 잘못된 위치는 `LOCALIZATION_INVALID` 정지로 처리합니다. 겹친 구간, 중복 이름, 잘못된 속도/설정 키는 시작 시 오류로 알립니다.

새 미션은 `missions.yaml`에 이름·구간·`allowed_states`·`speed_kmh`와 필요한 capability를 추가합니다.
`allowed_states`는 로드된 상태 중 제어 참여를 허용할 목록입니다. `pedestrian: true`는 보행자 예측,
`roundabout: true`는 기존 회전교차로 제어를 허용합니다. 센서 수신 자체는 미션별로 끄지 않습니다.
신호 상태 `STOP_LINE`은 카메라팀이 구현·로드하면 일반/상자/보행자에서 허용됩니다.
별도 신호 미션으로 분리할 때는 실제 정지선의 준비·해제 구간을 표에 넣고 해당 구간에만 허용하면 됩니다.
현재 카메라 미구현 상태를 FSM이 대신 구현하거나 활성화하지는 않습니다.

실행 예: `bash run_competition.sh mission_config:=/절대경로/missions.yaml`.
기존 개별 상태 시험은 `mission.enabled:=false` ROS 파라미터로 기존 방식에 비교할 수 있습니다.
상위 FSM이 켜져 있으면 속도 정책은 `missions.yaml`이 우선하며, `cruise.zone_*`는 단독/legacy 실행용입니다.
기존 `pedestrian_yield:=false`와 `roundabout_control:=false` 런치 스위치는 그대로 적용됩니다.

RViz 상태판은 `미션 / 하위상태`를 표시합니다. `/behavior/mission`에는 미션, 하위 상태,
차단 이유, 전환 이유, 상한을 JSON으로 발행하며 기본 bag에도 기록합니다.

### 미션별 RViz 표시

기본 화면은 지도·자차·실제 `/control/path`와 현재 미션의 판단 근거만 표시합니다.
상자에서는 회피 대상과 진입/복귀 위치, 보행자에서는 양보 대상·경로 충돌 영역·이동 방향을 표시합니다.
보행자 통과 확인 막대는 실제 연속 관측 시간으로만 채워집니다. 관측이 끊기면 `OBSERVATION MISSING`이며,
보이지 않는 것을 통과 완료로 표시하지 않습니다.

회전교차로 진입 전에는 기존 군집 경계/선택 틈/후행차 충돌 근거를 표시합니다.
진입 결단 이후에는 후행차 진입 판단 그림을 숨기고 전방 통로·관측된 표면·정지 여유 경계만 표시합니다.
주황 정지 여유 경계는 현재 거리 제어의 `STOP_CLEARANCE_M=0.8m`(앞 범퍼 기준)이며, 관측 표면은 평가 당시
경로 위치에 고정합니다. 관측이 오래되면 주황으로 표시합니다. 미래 자차 박스는 기본 화면에 추가하지 않습니다.

큰 상태판 한 개에 미션/하위 상태, 현재 속도→실제 제어 목표 속도, 판단 이유를 표시합니다.
`PEDESTRIAN`, `VEHICLE GUARD`, `OBJECT GUARD`, `OBSERVATION STALE`을 구분합니다.
고속도로에서도 제어 목표는 무한대가 아닌 추종기가 실제 발행한 속도입니다. 목표 속도/제한 종류는 새
`/control/telemetry`에서 받아 표시하며 0.5초가 지난 값은 숨깁니다. `PATH/STEERING`은 경로·조향 속도 정책,
`MISSION`은 행동 상한, `DR`은 추측항법 제한, `STOP/PATH`는 정지 요청이 포함된 경로 제어입니다.
터널에서는 GPS/DR도 함께 표시합니다. 표시용 데이터는 제어 판단에 다시 입력하지 않습니다.

각 MarkerArray는 새 화면마다 DELETEALL과 1초 수명을 사용해 지난 미션의 마커를 남기지 않습니다.
원시 검출 박스와 거리 눈금은 기본 화면에서 숨깁니다. 원시 박스는 RViz `Debug - all lidar objects`를 켜고,
상세 예측/거리 눈금은 `bash run_competition.sh rviz:=true visual_details:=true`로 확인합니다.
`/behavior/mission`, `/behavior/prediction`에 `yield_status`(대상 종류, 단계, 이유)를 추가했고
`/control/telemetry`도 기본 bag에 기록합니다. 화면에서 보이는 YIELD 이름만으로 보행자 양보라고 해석하지 않습니다.

### 음영 구간 회피와 보행자 통과 해제 (2026-09-25)

- `TUNNEL` 체크포인트(1875–2000m)는 `AVOID`를 허용한다. 위치는 기존 DR을 사용하며 카메라/라이다 위치 보정이나 라이다 빈 공간 판정은 추가하지 않았다.
- 기존 라이다 상자 검출과 `prediction.link_set`의 지도 링크 폭을 사용한다. 반대 차로도 포함한 도로 합집합 안에서 ±6m 후보를 검사하고, 통과 가능한 연속 오프셋 구간 중 넓은 쪽을 선택한다. 일반 상자 미션 후보는 기존 ±3m이다.
- 터널 진입 곡선 최소 12m. 복귀 길이는 `max(20m, 일반 복귀 길이) × sqrt(max(1, |오프셋| / lane_half_m))`로 횡이동량을 반영한다. 후보마다 이 길이로 지도상 차체 여유를 검사하고, 선택한 복귀 길이는 회피 중 유지한다. 차폭+0.3m 여유를 포함한 회전 차체를 지도상 샘플 검사한다. 링크 사이 측량 틈은 0.15m까지 허용하므로 도로 외곽의 유효 여유는 최소 0.15m이다. 지도는 실제 비어 있음의 보증이 아니며 DR 누적 오차를 보정하지 않는다.
- 지도 누락/통과 후보 없음은 정지 요청이다. 회피 시작 뒤 반대편으로 급전환하지 않는다. 터널/DR 추가 고정 속도 상한은 없다. RViz에 `MAP AVOID LEFT/RIGHT/STOP`을 표시한다.
- 보행자 몸체 전체가 자차 반폭+0.35m 밖이고 안쪽으로 돌아오지 않는 **새 관측(0.25초 이내)**이면 추가 0.35초 확인 대기 없이 양보를 해제한다. 다른 위협이 있으면 정지를 유지한다. 크기 미확인/오래된 관측/단순 소실은 기존 확인 절차를 사용한다.
- 실제 K-City 지도와 기록 경로의 s=1880/1900/1950에 20m 앞 상자를 가정한 정적 검증에서 왼쪽 +3.25m 후보를 확인했다. 시뮬레이터에서 음영 구간 상자를 놓고 실제 회피하는 검증은 별도 필요하다.


고속도로는 `allowed_states: [CRUISE]`만 허용합니다. 보행자/차량 YIELD, 상자 회피, 정지선, 터널 및 회전교차로 진입 제어가 개입하지 않습니다. 고속도로 NPC 대응은 전용 HighwayRuntime이 담당합니다. 제어기 자체의 경로 곡률 제한과 센서 이상 정지는 유지합니다. [고속도로 안내](../../docs/HIGHWAY.md)를 참고하세요.

## RViz 미션 체크포인트

`bash run_competition.sh rviz:=true`에서 `/behavior/markers`에 모든 미션 시작·종료 체크포인트를 표시합니다.
`config/missions.yaml`의 경로 거리로 위치를 계산하며 `CP 번호`, `미션 IN/OUT`, `s`를 표시합니다.
시작은 청록색, 종료는 회색, 같은 위치의 종료·시작은 보라색 한 지점으로 묶습니다.
현재 설정은9개 지점이며, RViz의 `mission_checkpoints` / `mission_checkpoint_labels` namespace에서 선·글자를 따로 끌 수 있습니다.

### 지도 체크포인트 표시

RViz의 `/behavior/markers`에서 FSM 진입·종료 경계는 굵은 보라색 횡선과 `FSM 미션 IN/OUT`으로 표시한다. 공식 체크포인트는 제공된 `checkpoints.txt`의 실제 좌표에 청록색 반경 3m 원과 `CP01` 등의 번호로 표시한다. CP11처럼 FSM 경계가 아닌 필수 통과 지점도 포함한다. FSM 경계 순번을 공식 CP 번호로 사용하지 않으며, 표시는 주행 판단에 영향을 주지 않는다.
