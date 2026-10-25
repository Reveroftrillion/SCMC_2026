# s315.6m 정적 구간 정지 — 2026-10-23

## 개발 목표·범위·완료 기준

사용자가 남은 개발 시작을 요청해 저장된 run-03의 s315.649m 정적 실패를
오프라인으로 분석한다. 브랜치는 `fix/static-stop-replay-20261023`이다.
기존 터널 수정·미추적·이동·문서·사용자 변경은 보존한다.

- 목표: 관측·지속 기억·고정 우회 경로 중 정지 경계를 확인하고 원인에 맞는 최소 수정.
- 영향 후보: LiDAR 지면/검출/clearance, planning의 BEV/StaticCourse 및 관련 검사.
  원인 확인 전 생산 코드·임계값·ROI·신호 기억·watchdog·속도 정책은 바꾸지 않는다.
- 방법: 실패 raw와 footprint/clearance 요청을 같은 stamp로 추적해 출생·마지막 관측·
  거부 상자를 연결하고, 성공 입력과 차이를 비교한다. 확인된 경계만 최소 수정한다.
- 검증: 실패의 사전 재현, 수정 후 동일 입력 및 실물 장애물/가림 보존,
  관련 회귀·필요한 전체 회귀·구문·문서 링크·차등 비교. 과거 기대값은 보존한다.
- 완료 기준: 재현 명령·원인·최소 변경·검사 결과·남은 호환성 기록.
  오프라인 결과와 이후 실제 주행 검증을 구분한다. 이 작업에서는 주행을 시작하지 않는다.

## 기준 입력·보존

원본 실패는 `docker/artifacts/ctrl-start-validation-20261023-125934/run-03/diagnostics_0.bag`,
비교 성공은 같은 루트의 run-04다. 앞선 정지·완주 결과를 수정하지 않는다.
새 작업 자료는 `docker/artifacts/static-stop-20261023-140321/`이다.
시작 src/tools SHA256·tar.gz·Git 상태·기존 src patch를 여기에 별도로 저장했다.
시작의 ground.py/obstacle_node.py/test_ground.py는 앞선 터널 수정이며 이번 변경과 분리한다.

## 사전 재현과 원인 경계

기존 footprint를 target tick으로 재생하면233 tick 중191 tick에서 cap0이 나고,
mapped pavement는 통과하지만 observed box conflicts with fixed course가 남는다.
기억 ID301/326은 s335~341m의 상자를 만들었고 마지막 관측은 제동 중이었다.
같은183 scan 연속 재생에서도 비슷한 정지·기억이 생겼다.

출생 스캔1792730259에서 지면 추정은 None이다. 아스팔트 지지833개에 대해
현재 median reference_z=0.738726m, RANSAC의 지지809개를 재추정한 중심 절편은
-0.296567m가 되어 기존 ±0.25m guard에서 탈락한다. 아스팔트 최소제곱 원점 절편은
0.431393m, 기울기는 약[0.024272,0.001545]다. 같은 점에 원점 절편으로 높이 기준만
옮기면 지지833개·동일 기존 guard를 통과한다. 도로의 x방향 기울기를 높이 중앙값에
섞어 기준을 옮긴 것이 실패 조건이다. raw 관측의 바닥점은 제동 중 약0.94m로 보인다.

기억을 지우는 기준을 완화하는 대신 지면 추정의 기준점 문제를 수정한다.
기존 median 방식이 성공하면 그대로 반환하고, 실패했을 때만 같은 아스팔트로
추정한 원점 절편으로 다시 중심을 맞춘다. 기존 RANSAC 기울기/절편/지지점/잔차/
공간 폭 검증을 모두 다시 적용한다. 평면을 직접 무검증 채택하지 않는다.
영향 파일은 ground.py와 신규 회귀 검사이며 기존 obstacle_node·BEV/StaticCourse·
clearance·신호·속도·ROI·watchdog는 이번에 변경하지 않는다.

첫 기억 진단 도구는 numpy JSON 직렬화에서 실패했다. 진단 출력의 배열 변환과
시점별 deep copy만 보완해 새 memory-before-v2에 재실행했다. 기존 실패 폴더와
입력·정지 판정은 변경하지 않았다.

## 최소 수정

`ground.py`의 estimate_material_ground_plane만 수정했다. median 방식의 성공 결과는
그대로 반환한다. 실패하면 같은 아스팔트 점들의 최소제곱 원점 절편으로 높이 기준만
다시 맞추고 **기존 estimate_ground_plane 검증을 다시 통과해야** 평면을 채택한다.
기준점 재계산 값을 평면으로 직접 사용하거나 검증을 우회하지 않는다.

기울기0.18·절편0.25·잔차0.12·지지점/지배율·폭/ROI·높이창을 바꾸지 않았다.
BEV 기억/clearance, 고정 우회 경로, 신호·속도·watchdog도 그대로다.
신규 test_material_ground_reference.py에 재현·낮은 물체 보존·기존 성공값 동일·
무지지/희소/좁은/과도한 기울기/비평면 거부 검사4개를 추가했다.
기존 test_ground.py 등 기대값은 변경하지 않았다.
시작 스냅샷과 비교한 기존 소스 변경은 ground.py1개이며 새 검사 파일1개다.

## 오프라인 검증

| 검사 | 결과·범위 |
| --- | --- |
| 신규 검사로 수정 전 소스 확인 | 의도한 회귀1 FAIL, 나머지3 PASS. median 기준의 실패를 잡음 |
| 기존 지면+신규 집중 검사 | 15 PASS |
| 실패 raw 연속 재생 |183스캔 중 STATIC99스캔. 평면None16→0, cap0 77→0 |
| 실제 상자·미관측 영역 | 실제 상자 최종 footprint 유지. 미관측에 대한 감속·stop_s 요청 유지 |
| 이전 성공 raw 재생 |618스캔의 target618개 전부 정확히 동일. 전후 cap0 모두0 |
| 앞선 터널 입력 영향 | 기존 부분 replay와 같은 경계. 실제 긴 정지 이력/새 궤적의 동등성 증거는 아님 |
| 전체 알고리즘 |Docker Python3.10·격리 src·ROS 대역:950 PASS, FAIL0/skip0, 경고10,333.93초 |
| 도구 무결성·launch 해석 |9 PASS, FAIL0/skip0, 외부 deprecation 경고1 |
| 입력 보존 |실패 추출196개·성공 추출656개 manifest SHA256 일치. raw bag별 hash도 새 기록 |
| 비교 원본 |앞선 보존 manifest의530개 파일 불변 |

집중15개는 전체950개와 중복되므로 합산하지 않는다.
[공유 요약](static-stop-20261023/summary.json),
[도구 검사](static-stop-20261023/tools-summary.json)를 참고한다.
시점 순서·pose는 기록 궤적을 고정했고 cloud당1회 계산했다. 실제20Hz executor·
신호/이전 모든 미션 기억·수정 후 폐루프 궤적을 완전히 재현한 것이 아니다.
성공 raw에서 perception 필드 전체가 동일하다는 뜻도 아니다. target618개를 비교했다.

일반 diff --check는 기존 CRLF 파일의 추가 줄에서 CR을 whitespace로 보고한다.
기존 줄바꿈을 유지하고 CRLF-aware diff 검사를 사용한다. 새 검사 파일은 LF다.
첫 집중 검사 명령은 컨테이너 작업 경로에 ros1-port를 중복 지정해 exit4/no tests였고,
올바른 src 경로와 새 basetemp로 재실행해15 PASS를 확인했다.
도구/경로 오류를 runtime 회귀나 성공으로 합산하지 않는다.

## 재현·소스 복원 기준

기존 자료는 덮어쓰지 않는다. 아래 출력은 반드시 새 이름을 사용한다.

```bash
# 저장소 루트: 순수 계산 검사. 실제 ROS 노드/제어를 시작하지 않음
bash ros1-port/docker/run.sh env PYTHONDONTWRITEBYTECODE=1 \
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=/workspace/ros1-port/src/foscar_lidar \
  python3 -m pytest -p no:cacheprovider -q \
  src/foscar_lidar/test/test_ground.py \
  src/foscar_lidar/test/test_material_ground_reference.py

# 컨테이너 안: 저장된 실패 입력 재생
python3 tools/replay_lidar_sequence.py replay \
  --input docker/artifacts/static-stop-20261023-140321/sequence \
  --output docker/artifacts/static-stop-20261023-140321/new-replay \
  --focus-x=-56 --focus-y=-134 --behavior
```

`source-before.tar.gz`는 이번 작업 시작의 src/tools 원문이며 앞선 터널 수정도 포함한다.
`baseline/`은 이를 격리 복원한 사본이다. `static-only.patch`는 그 사본 대비 이번 변경만
포함한다. `source-delta.patch`는 Git HEAD 대비이므로 이전 터널 수정도 포함됨에 유의한다.
실패/성공 추출 입력·수정 전후 재생·reference-probe.json·regression.log·tools.log를
함께 사용한다. 현재 소스와 원본에서 캐시/테스트를 생성하지 않도록 격리 사본을 사용한다.

## 남은 실제 검증

이번 작업에서는 MORAI·UDP 제어·ROS master를 실행하지 않았다.
최근 완주 run-04는 **이번 정적 기준점 수정 전** 결과다. 현재 소스의 실제 성공으로
표현하지 않는다. 다음은 START Reset·Manual/P·연결 상태를 확인한 뒤
수신전용 검증과 감독 실제 주행으로 정적 접근·우회·터널·종료를 확인하는 것이다.

Windows·catkin 재빌드/설치 공간·실제 executor/부하·공식 차선/감점·다른 배치는 미검증이다.
기존 원본/리소스/식별자 기준 실패, 입력 중단과 카메라 종료 예외는 해결 처리하지 않는다.
기존 변경과 출처·라이선스를 보존했고 commit/push/merge는 하지 않았다.
