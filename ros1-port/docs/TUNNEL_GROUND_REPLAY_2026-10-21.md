# 터널 앞 정지 오프라인 재현 — 2026-10-21 KST

## 작업 계획

- 브랜치: `fix/tunnel-ground-replay-20261021`, 기준 HEAD는 작업 시작 시 기록한다.
- 목표: 2026-10-16 원본 무개입 시험의 터널 앞 정지를 점군→지면 처리→객체→회피 판단 경계로 재현한다.
- 영향 파일: 새 재현 도구·필요한 회귀 검사·이 문서·새 Git 제외 artifact. production 수정은 확인된 원인에 한정한다.
- 구현 방법: 기존 bag와 별도 정적 snapshot을 구분해 같은 입력의 높이·평면 거부 조건·인지 출력을 비교한다. 근거 없는 센서 장착 보정이나 임계값 변경은 하지 않는다.
- 검증 방법: 입력/코드 해시, 기존 지면·터널 인지 회귀 검사, 정지 프레임 및 실제 장애물 입력 비교. 실제 ROS 전송 검증과 ROS 대역을 사용하는 계산 재현은 구분한다.
- 완료 기준: 실패 경계와 재현 명령을 기록하고, 입증된 수정이 있으면 같은 입력과 기존 회귀 검사로 비교한다. 좌표 보정 등 근거 부족 항목은 런타임 수정 보류 사유를 남긴다.
- 범위: 실제 주행·제어 송신·MORAI 장면 변경 없음. 원본 비교 프로젝트·기존 JSON/manifest/날짜별 기록 보존.

## 판정

**지면 추정 실패와 가까운 도색 오검출을 재현하고 제한적인 수정안을 검증했다.
터널 앞 정지 해결은 아직 아니다.** 같은 위치의 새 플래너 상태를 사용한 비교에서도
수정 후 `PLANNER HOLD / v_max=0`이 남는다. 실제 주행은 수행하지 않았다.

기준 HEAD는 `0fe5ae635f5acbc79aae9e9c7f88a3474a63b233`이다.
2026-10-16 기록이 설명하는 당시 기준 커밋과 이번 작업 시작 HEAD를 구분한다.

## 재현한 경계

원본 무개입 bag의 record time `1792137137.5724103`, cloud stamp
`1792137137.5703506`, 후륜 s=1831.754474m의7,523점을 추출했다.
이 입력의 직전 위치·미션·footprint를 사용했으며, native rosbag 읽기 후 인지 계산에는
저장소의 명시적 ROS 대역을 사용했다. ROS 전송·스케줄링 검증은 아니다.

정지 프레임과 기존 별도 정적 snapshot `tunnel-cloud.npz`(stamp1792137300.6694,
7,525점)는 서로 다른 입력이다. 최초 비교는 위치 버퍼 없이18개 원시 검출을 보았으며,
최종 비교에서는 추출 위치를 odometry 버퍼에 넣어 실제 지도 점 필터까지 적용했다.
최종 HEAD 재현의15개 검출은 기록된15개 객체 중심과 최대 약0.000199m 오차로 대응한다.

기존 지면 추정은 최종 최소제곱 평면의 높이 절편이 약0.443m여서 ±0.25m guard에
거부된다. 지지점과 관측 폭을 확보한 뒤 최종 재추정에서 실패한 것이다.
지도 필터와 결합한 재현에서는 도색 반사도 범위의 전방32점이 모두 기존0.4m 높이
조건을 넘는다. 재료 반사도만으로 모든 검출의 물체 정답을 확정하지는 않았다.

임의 높이 이동 -0.53/-0.86m는 별도 진단 JSON에만 남겼다. 장착 좌표나 보정값으로
채택하지 않았다. 정지 프레임에 효과가 있는 이동도 다른 터널 프레임에서는 기존
평면 추정을 복구하지 못했다.

## 수정안과 보존 범위

- [ground.py](../src/foscar_lidar/foscar_lidar/ground.py)에 아스팔트 재료 기반 보조 평면 추정을 추가했다.
- [obstacle_node.py](../src/foscar_lidar/foscar_lidar/obstacle_node.py)는 **신선한 TUNNEL 미션·intensity 존재·기존 추정 실패** 때만 보조 추정을 사용한다.
- 이미 아스팔트로 처리하던 반사도 범위의 점을 중앙 높이로 옮긴 후 기존 추정기의 지지점 수·우세 비율·기울기·관측 폭 검사를 그대로 적용하고 높이 기준을 복원한다.
- 센서 TF·장착 파라미터·높이 임계값·ROI·28m/좌우5m 보정 범위·신호·watchdog·플래너는 변경하지 않았다.
- 아스팔트 근거가 부족하거나 평면 조건을 만족하지 않으면 기존 고정 높이 처리로 돌아간다. 다른 미션의 보조 추정은 추가하지 않았다.

이 변경은 터널에서 이미 사용하는 시뮬레이터 재료 분류에 의존한다. 다른 센서의
반사도 체계나 잘못 분류된 재료까지 검증한 변경은 아니다. 센서 원점과 후륜축 좌표의
동등성은 여전히 확인되지 않았다.

## 같은 입력 비교

`replay_lidar_ground.py --baseline-head`는 기준 HEAD의 ground/obstacle 모듈을 메모리에
읽는다. checkout이나 작업 파일 교체 없이 수정안과 동일한 cloud·위치·미션·지도를
비교하고 소스 SHA256을 결과에 기록한다. 원본 비교 프로젝트를 실행하지 않는다.

| 입력 | s(m) | 검출 수 HEAD→수정 | 보정 범위 내 도색 후보 중 높이 gate 이상 HEAD→수정 |
| --- | ---: | ---: | ---: |
| 원본 정지 프레임 | 1831.75 | 15→7 | 32→0 |
| 별도 보조 구간 중간 | 1891.67 | 4→0 | 17→0 |
| 별도 보조 구간 후반 | 1941.64 | 12→7 | 19→0 |
| 별도 보조 구간 출구 부근 | 1969.40 | 11→7 | 18→0 |
| 원본 정적 장애물 접근 | 302.00 | 6→6, 검출 배열 전체 동일 | 181→181 |

검출 수 감소만으로 정상 판정을 내리지 않았다. 별도 합성 회귀에서0.7m 장애물의
상대 높이와 검출이 유지되고 도색 조각이 제거되는지 확인했다. 희소·좁은·과도한
기울기·높이 범위 밖·비유한 입력, 다른 미션·만료된 미션의 기존 처리도 검사했다.
실제 터널 장애물 전체에 대한 정답 검증은 아니다.

## 남은 정지 경계

기존 route·지도·avoid 설정과 재현 footprint를 넣은 **새 상태의 정적 플래너 probe**에서
HEAD의 `AVOID / v_max=0 / stop_s=1843.282506797338 / planner hold`가 기록값과 일치했다.
수정 출력에서는 계획 대상 상자가3개에서1개로 줄었지만,
`v_max=0 / stop_s=1858.8660824760323 / planner hold`로 여전히 멈춘다.

남은 핵심 검출은 base_link 전방 약28.77m/횡1.89m, 꼭대기 약0.733m이며
경로상 상자 s[1861.9,1862.2]/d[+0.20,+0.55]를 만든다.
기존 보정 범위28m 밖이어서 이번 평면으로 높이를 외삽하지 않았다.
플래너에는 횡offset +2.5~+3.5m 후보가 있지만 도로 내 차체 곡선 검사에서 거부된다.
이 물체의 실제 대응과 곡선 거부 원인을 확인하기 전에 범위·지면 임계값·도로 조건을
완화하지 않았다. 전체 추적 기억을 복구한 시간 순서 replay는 추가로 필요하다.

## 검증·기록

- Docker Python3.10, ROS 대역 사용: 최종 **142 PASS, FAIL0, skip0**. 전체 LiDAR 검사, 터널 slalom/회피, 진단 도구 검사를 포함한다.
- 최초 새 진단 단위 검사1개는 완전한 높은 평면도 최종 refit까지 간다는 잘못된 가정으로 실패했다. 기존 기대값을 바꾸지 않고 새 검사 가정을 바로잡았다. 최초 실패·중간67/141 PASS·최종142 PASS 로그 모두 보존했다.
- 새 도구의 입력 비변형·trace 복원·기존 결과 덮어쓰기 거부·HEAD 모듈 SHA256 일치를 검사했다.
- 구문 검사와 입력 SHA256 전후 비교 PASS. 원본 비교 프로젝트의 tracked diff 없음. 새 문서 내부 링크 확인.
- 일반 `git diff --check`는 기존 CRLF 형식을 유지한 파일의 추가 줄에서 CR을 trailing whitespace로 보고한다. CRLF를 허용한 `git -c core.whitespace=cr-at-eol diff --check`는 PASS. 기존 파일 전체 줄바꿈 변환은 하지 않았다.
- 전체 차등 비교, Windows 전용 wrapper, native ROS launch 재실행, 실제 주행·정지 해제·완주 검증은 이번 수정 후 실행하지 않았다.

근거 루트는 Git 제외 `docker/artifacts/tunnel-ground-replay-20261021/`이다.
`summary.json`, `input-sha256.json`, `*-mapped-head.json`, `*-mapped-fixed.json`,
`planner-probe.json`, `probe_planner.py`, `final-regression.log/json`을 참조한다.
큰 bag·원본 입력·기존 JSON은 덮어쓰지 않았다. commit/push/merge 없음.
이번에는 ROS launch·주행·녹화 프로세스를 기동하지 않았고 MORAI를 변경하지 않았다.

## 재현 명령과 다음 단계

저장소 루트에서 다음 명령을 사용한다. 출력 폴더는 매번 새로 만든다.

```bash
rmt_ground_trial=$(mktemp -d ros1-port/docker/artifacts/tunnel-ground-XXXXXXXX)
bash ros1-port/docker/run.sh python3 tools/replay_lidar_ground.py extract \
  --bag docker/artifacts/fullcourse-20261016/baseline/diagnostics_0.bag \
  --at 1792137137.57 --output "${rmt_ground_trial#ros1-port/}/frame"
bash ros1-port/docker/run.sh python3 tools/replay_lidar_ground.py inspect \
  --cloud "${rmt_ground_trial#ros1-port/}/frame/cloud.npz" \
  --context "${rmt_ground_trial#ros1-port/}/frame/context.json" \
  --baseline-head --output "${rmt_ground_trial#ros1-port/}/head.json"
bash ros1-port/docker/run.sh python3 tools/replay_lidar_ground.py inspect \
  --cloud "${rmt_ground_trial#ros1-port/}/frame/cloud.npz" \
  --context "${rmt_ground_trial#ros1-port/}/frame/context.json" \
  --output "${rmt_ground_trial#ros1-port/}/fixed.json"
```

다음 단계는 남은28.77m 검출을 영상·반사도·연속 점군과 대조하고, 그 위치의 도로 곡선
거부를 좌표와 차체 경계로 분해하는 것이다. 이 수정안을 터널 정지 해결이나 주행 가능한
수정 완료로 처리하지 않는다. 사용자의 실제 주행 승인은 현재 범위에 포함되지 않는다.
