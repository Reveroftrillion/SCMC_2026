# 터널 잔여 객체·회피 곡선 연속 재현 — 2026-10-21 KST

## 계획

- 목표: 전방 약28.77m 검출의 지속성과 실제 대응, 회피 곡선의 도로 거부 지점을 확인한다.
- 브랜치: `fix/tunnel-ground-replay-20261021`. 기존 지면 수정안·미추적 문서·검증 기록을 보존한다.
- 영향 파일: 연속 입력 추출·재현 도구, 회귀 검사, 이 문서, 새 Git 제외 artifact. runtime 변경은 재현 근거가 확보된 부분에 한정한다.
- 방법: native rosbag에서 입력을 새 폴더로 추출하고, ROS 대역에서 기록 시각 순서대로 위치·미션·점군을 재생한다. 추적·측정 벽·플래너 기억을 유지하고 차체 곡선 검사 실패점을 기록한다.
- 검증: 이전 HEAD/기존 지면 수정안/추가 수정안의 같은 입력 비교, 원본 입력 해시 보존, 관련 회귀 검사와 구문·문서 링크·diff 검사.
- 완료 기준: 잔여 객체 및 도로 거부 경계를 정량화하고, 입증된 변경은 같은 연속 입력과 기존 회귀 검사로 검증한다. 추정·미확인·실제 ROS/주행 미검증을 구분한다.
- 실제 주행·제어 송신·MORAI 장면 변경·push/main 병합은 수행하지 않는다.

## 진행 기록

이전 단일 프레임 probe는 추적 이력·누적 벽과 일부 실제 Context 설정을 복구하지 않은
제한적인 계산이었다. 이번에는 실제 callback 순서와 누적 상태를 더 가깝게 복원한다.

## 결과와 이전 분석 정정

**관측된 터널 앞 정지 입력에서는 최소 수정 후 PLANNER HOLD가 사라졌다.**
접근214프레임·정지78프레임을 추적·벽·회피 기억을 유지하며 계산했다.
추가로 CRUISE/AVOID/TUNNEL 상태를 합성한 Docker Python3.10 계산에서214개 모두
target을 생성했고, 그중 TUNNEL 소유160개는 모두 `v_max=13.88888888888889m/s`였다.
이는 고정된 기록 궤적에서의 계산 결과다. 실제 출발·터널 통과·전체 완주 검증은 아니다.

[앞선 기록](TUNNEL_GROUND_REPLAY_2026-10-21.md)의 **잔여 객체 연결에는 오류가 있었다.**
전방28.77m/횡+1.89m 검출은 map 약(41.15,-488.09)m이며, 곡선 경로 기준
s≈1858.71m/d≈+7.21m이다. 플래너를 막던 s≈1862m/d≈+0.34m 상자와 같은 객체가 아니다.
실제 잔여 정지 객체는 **전방29.65m/횡-5.68m**, map 약(34.39,-484.57)m의 얇은 검출이다.
과거 기록은 당시 분석으로 보존하고 이 문서에서 정정한다.

## 확보한 근거

### 잔여 검출과 높이 경계

정지 입력에서 잔여 검출을 만드는3점은 intensity가 모두110이다. 이는 기존 코드가
도색 후보로 취급하는45~112 범위에 들지만, 기존 높이 약0.630~0.631m가
paint gate0.4m보다 높아 남는다. 같은 스캔의 재료 평면을 적용하면 약0.0066~0.0071m다.
이 값은 기존 도색·지면 제거 경계로 제거되는 높이다.

기존 보정 사각형은 전방3~28m/횡±5m다. 도로가 휘면서 실제 경로상 전방 도색이
차체 좌표의 전방29.65m/횡-5.68m로 나타나 두 조건 모두 벗어난다.
앞선 전방 거리만 연결한 시험에서는28.77m 검출은 사라졌지만 실제 정지 객체는 남았다.
그 실패 입력과 수정 diff는 별도로 보존했다.

대표 스캔에서 기존 ROI 안의 아스팔트 후보1,354점 중1,082점이 기존0.12m 잔차
조건으로 근거리 평면과 일치했다. 이 일치점들의 실제 XY convex hull 안에
정지 객체3점도 포함된다. 따라서 ROI나 높이 임계값을 일괄 완화하지 않고
**현재 스캔의 관측 지지 영역 안에서만** 같은 높이 기준을 적용할 근거를 확보했다.
재료 반사도와 기하 근거로 코드의 노면 처리 경계를 확인한 것이며,
공인 물체 정답·충돌 판정 채널로 모든 검출의 실제 물체 종류를 확정한 시험은 아니다.

같은 위치 재시작 시험의10초 입력86프레임도 추출·재현했다. 이 구간은 s≈1838.62m이며
관심 지점 반경1.5m에 직접 대응하는 점이 없었다. 따라서 이 시험에서 같은 물체의
가까운 관측 높이를 확인했다고 주장하지 않는다.

### 회피 곡선 거부

위 상자를 피하는 +2.5m 후보 곡선은 차체의 여유 영역을 포함한 도로 검사에 실패한다.
대표 첫 실패는 후륜 s≈1831.75m에서 차체 전방2.255m/좌측1.746m의 점이다.
map 약(55.90,-466.06)m, 경로 s≈1833.91m/d≈+1.92m이며 지도 밖이다.
같은 후보의 거부138점 모두 기본 지도에서도 밖이어서 측정 벽만의 문제로 볼 수 없다.
곡선 진행 중 도로 사이 빈 영역도 통과하므로 첫 점만 유예한다고 해결되지 않는다.

도로·차체 검사 완화는 하지 않았다. 원인이 된 노면 검출을 제거한 뒤에는
이 불필요한 회피 후보를 만들 필요가 없어 해당 거부 경로로 들어가지 않는다.
실패점은 artifact의 `road-failures.json`, `geometry.json`, `geometry.png`에 있다.

## 최소 수정

- [ground.py](../src/foscar_lidar/foscar_lidar/ground.py): 기존 터널 아스팔트 평면에 대한 현재 스캔의 지지 영역을 계산하는 `material_height_region` 추가. 기존 지지점150/120, 우세60%, 잔차0.12m, XY 폭8m/3m 조건을 재사용한다. 기존 사각형 밖에도 넓은 지지가 있어야 추가 영역을 인정한다.
- 추가 영역은 일치하는 아스팔트 점들의 convex hull과 기존 ROI 안으로 제한한다. 외삽 거리 상수나 새 파라미터는 추가하지 않았다. 근거가 부족하면 기존 사각형으로 돌아간다.
- [obstacle_node.py](../src/foscar_lidar/foscar_lidar/obstacle_node.py): 기존 평면 추정이 실패하고 신선한 TUNNEL 미션·intensity가 있을 때만 보조 평면과 관측 지지 영역을 사용한다.
- `relative_height`는 내부 선택적 mask 인자를 추가했다. 기존 호출의 기본3~28m/횡±5m 동작은 그대로다. 원본 PointCloud2는 변경하지 않는다.
- TF·센서 장착값·ROS 메시지/토픽/파라미터·launch/YAML·ROI·높이/반사도 threshold·신호·watchdog·속도 제한·플래너 코드는 변경하지 않았다.

## 연속 입력 비교

입력은 기존 bag를 읽기 전용으로 열어 추출했다. 실제 메시지 생성·ROS master·
UDP 소켓·제어 송신 없이 ROS 대역에서 callback 계산과 객체 추적을 수행한다.
기록된 odometry 속도 이력으로 body pitch를 계산하고, 미션 freshness는 기록의
수신 간격으로 재현했다. 측정 벽 기억과 새 플래너의 clearance query 피드백도 포함한다.

| 기록 입력·계산 방식 | 프레임 수 | HEAD PLANNER HOLD | 직전 근거리 수정안 | 최종 관측 hull 수정안 |
| --- | ---: | ---: | ---: | ---: |
| 접근25초, AVOID 단독 계산 | 214 | 157 | 127 | 0 |
| 정지9초, AVOID 단독 계산 | 78 | 이번 구간 HEAD 별도 실행 안 함 | 76 | 0 |

숫자는 프레임 수이며 독립 실패 횟수가 아니다. 접근 입력은 s1704.61→1831.75m이고,
정지 입력은 s1831.75m 부근이다. 모든 수정안에 동일한 기록 위치를 사용했다.
최종 정지 입력에서는 플래너 상자가0개이며 AVOID 진입 조건이 성립하지 않는다.
단독 probe의 target=None은 **AVOID 목표가 없다는 뜻**이다.
전체 목표 발행 중단과 구분하기 위해 상태 합성 probe를 별도로 추가했다.

Docker Python3.10에서 동일한 접근 입력을 CRUISE/AVOID/TUNNEL로 합성하면
HEAD는156프레임에서AVOID, 마지막 target은v_max0/stop_s1843.282506797338이다.
최종 수정안은214프레임 모두CRUISE이며 TUNNEL 구간160개는50km/h 미션 상한을 유지한다.
HIGHWAY Runtime/YIELD/신호·후속 follower는 이 probe에 포함되지 않으므로,
앞부분 HIGHWAY 구간의 전체 속도 정책이나 보행자 우선권을 검증한 결과가 아니다.

호스트 Python3.8과 Docker Python3.10의 정지78프레임에서 stamp·s·pose·plane·검출·
footprint·target·box·reason 배열은 정확히 같았다. 도구에 mission 필드를 추가한 전후
기록이라 JSON 키 전체 비교와 이 공통 계산 필드 비교를 구분한다.

## 테스트와 보존

- Docker Python3.10/ROS 대역: **148 PASS, FAIL0, skip0**. 전체 LiDAR·터널 slalom/회피·두 재현 도구 검사를 포함한다. 중간145/147 PASS 로그도 보존했다.
- 30m 앞 및 횡±5m 사각형 밖의 합성0.7m 장애물이 유지되는지 검사했다. 지지 영역 바깥·희소·좁은·불일치·비유한·기존 ROI 밖 근거는 보정을 확대하지 않는지 검사했다.
- 실제 정적 장애물 접근 프레임 s≈302m의 검출6개는 HEAD와 최종 수정안의 전체 검출 배열이 정확히 같다. 다른 미션·만료된 미션·intensity 부재의 기존 처리도 관련 회귀 검사에 포함된다.
- 추출 입력 manifest SHA256 전후 일치, 구문 및 새 문서 링크 검사 PASS. 기존 원본 비교 프로젝트 tracked diff 없음.
- 요약 스크립트 최초 실행은 JSON의 비유한 수 표현 문자열을 숫자로 바로 비교해 실패했다. 원래 기록을 보존하고 문자열을 수로 해석하도록 요약 도구만 바로잡았다. runtime·기존 기대값은 변경하지 않았다.
- 일반 `git diff --check`는 기존 CRLF 파일의 추가 줄에서 CR을 trailing whitespace로 보고한다. 기존 형식을 보존했고 `git -c core.whitespace=cr-at-eol diff --check`는 PASS다.
- 실제 ROS executor/전송 주기, 전체 bag·추적 이력, 전체 미션 차등 비교, 센서 원점/후륜축/clearance 높이 좌표 동등성, 수정 후 실제 주행·출발·충돌·완주는 미검증이다.

처음부터 전체 주행 이력을 복원한 replay는 아니다. 추적·벽 기억은 각 입력 구간에서
시작하고, 플래너는 cloud당1회 평가한다. 원래20Hz executor의 지연과 모든 callback
동시성을 재현하지 않는다. 계획이 달라져도 위치는 기존 기록을 따르는 계산이므로
이를 수정 후 실제 차량 궤적이나 완주로 해석하면 안 된다.

## 파일·재현·인계

기존 작업 브랜치를 유지했다. 기존 지면 수정안과 사용자 미추적 문서를 보존했으며
commit/push/merge는 하지 않았다. 이번 작업의 새 도구는
[replay_lidar_sequence.py](../tools/replay_lidar_sequence.py), 검사 도구는
[test_replay_lidar_sequence.py](../tools/test_replay_lidar_sequence.py)다.
ground/test_ground와 obstacle/test_tunnel_intensity에 근거가 확보된 변경·검사를 추가했다.

새 원시 자료 루트: `docker/artifacts/tunnel-sequence-20261021/`.
`summary.json`, `start-state.json`, `incoming-changes.patch`, `range-only-candidate.patch`,
`approach/`, `stopped/`, `restart-stopped/`, 각 재현 폴더의 `frames.jsonl` 및
`summary.json`, `final-regression.json/log`를 참조한다. 큰 입력과 artifacts는 추적하지 않는다.

저장소 루트에서 새 폴더로 재현한다. 바인딩·주행 권한 옵션은 필요 없다.

```bash
rmt_seq=$(mktemp -d ros1-port/docker/artifacts/tunnel-seq-XXXXXXXX)
bash ros1-port/docker/run.sh python3 tools/replay_lidar_sequence.py extract \
  --bag docker/artifacts/fullcourse-20261016/baseline/diagnostics_0.bag \
  --start 1792137025 --end 1792137050 --output "${rmt_seq#ros1-port/}/input"
bash ros1-port/docker/run.sh python3 tools/replay_lidar_sequence.py replay \
  --input "${rmt_seq#ros1-port/}/input" --output "${rmt_seq#ros1-port/}/head" \
  --baseline-head --behavior --focus-x 34.391208 --focus-y=-484.573875
bash ros1-port/docker/run.sh python3 tools/replay_lidar_sequence.py replay \
  --input "${rmt_seq#ros1-port/}/input" --output "${rmt_seq#ros1-port/}/fixed" \
  --behavior --focus-x 34.391208 --focus-y=-484.573875
```

다음 검증은 실제 터널 장애물 정답이 있는 추가 기록과 더 긴 추적 구간이다.
현재 상태는 **해당 정지 입력의 오프라인 회귀 해소**, 실제 주행 검증 전 수정안이다.
이번 작업에서는 ROS launch·주행·녹화를 기동하지 않았으며 MORAI를 변경하지 않았다.
