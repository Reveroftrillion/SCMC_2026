# practice/ — 연습 전용 (제출 전 반드시 삭제)

**이어받는 에이전트는 [HANDOFF.md](HANDOFF.md) 부터 읽는다** (규정·환경·러너 사용법·현재 상태·함정).

대회 규정상 사용할 수 없는 시뮬레이터 채널(충돌 데이터, 시나리오 로드, ObjectInfo,
EgoVehicleStatus 확장 등)을 쓰는 도구만 여기에 둔다. 주행 코드(`src/`, `run_*.sh`,
`status.sh`)는 이 폴더를 import·실행·참조하지 않는다. `COLCON_IGNORE` 로 빌드에서도 빠진다.

| 경로 | 용도 | 쓰는 채널 |
|---|---|---|
| `sim/sim_scenario.py` | 저장된 시나리오 로드(자차·NPC·장애물 초기화) | 시뮬 9095 송신 |
| `sim/record_collision.py` | 충돌 패킷 기록 | 9092 수신 |
| `avoid_test/` | 상자·보행자 회피 반복 시험 러너·모니터·분석 | 9092 수신 |
| `sim/loop_runner.py` | 시나리오 로드 → 주행 → 판정 반복. 정답 위치로 추측항법 오차, 횡가속도·슬립각 기록 | 9095 송신, 9092·7505·9111 수신 |
| `scenarios/` | 체크포인트별 시나리오 생성기와 결과, 시나리오 네트워크 템플릿 | – |
| `sim/analysis/` | bag 분석: 속도 제한 원인, 고속도로 앞차 추종·차선변경 허가, 60 km/h 규정 확인 | – (bag 만 읽음) |

시나리오 네트워크 템플릿(`scenarios/EgoNetwork_template_MN.json`)은 시뮬 PC
`...\Scenario\R_KR_PR_K-city_2025\EgoNetwork\<시나리오>_MN.json` 으로 복사해 쓴다.
EgoVehicleStatus(770)는 시뮬 송신 포트를 **9106** 으로 둔다 (9101 은 GPS 가 이미 사용 → 충돌해 안 나왔음).

## 제출 전

```bash
bash practice/check_submission.sh   # practice/ 밖에 연습용 채널 흔적이 없는지 확인
git rm -r practice CLAUDE.md && git commit -m "chore: remove practice tools and agent notes for submission"
```
