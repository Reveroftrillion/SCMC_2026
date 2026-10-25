# CTRL 팀 — SCMC 2026 ROS1 자율주행

CTRL 팀의 MORAI 자율주행 개발 저장소다. 현재 실행 코드는 `ros1-port/`에 있고, 외부 참고 코드의 ROS1 이식·수정본과 새로 작성 중인 제어 모듈을 함께 관리한다.

## 현재 개발 상태

| 구분 | 상태 |
| --- | --- |
| 현재 실행 스택 | 기존 ROS1 이식·수정본. 실행 패키지/모듈 이름은 `ctrl_*` |
| 새 제어 계산 | 조향·진행 구간·경로 변환·요청 수명 관리까지 오프라인 구현 |
| 새 계산 검증 | 단위 278개 통과. 단계별 합성 결과는 각 결과 문서 참조 |
| 실제 실행 연결 | 병행 계산 연결·기록 재생·Linux/ROS/MORAI 검증이 남음 |

새 제어 계산의 합성 통과를 기존 실행 스택의 완주 기록과 합산하지 않는다. 현재 launch는 계속 기존 제어기를 사용한다.

## 이번까지 반영한 개발

| 구성 | 구현한 역할 | 단위 검사 |
| --- | --- | ---: |
| `ctrl_lateral.py` | 경로 투영·목표점·조향각과 명령 제한 | 115 PASS |
| `ctrl_progress.py` | 진행 구간·재획득·위치 점프·시각 검사 | 47 PASS |
| `ctrl_path_conversion.py` | 일반/회피/local reference 변환과 전역 거리 대응 | 59 PASS |
| `ctrl_path_selection.py` | 요청 선택·유지·해제·만료와 오류 차단 | 57 PASS |

최신 결합 단위 검사는 **278 PASS, 실패·오류·skip 0**이다. 단계별 연속 검사는 조향 단독 10개 장면/6,600 step, 진행+조향 10개/6,600 step, 경로 변환 연결 6개/5,550 step, 요청 수명 연결 2개/2,500 step을 통과했다. 이들은 겹치는 합성 조건을 포함하므로 독립 주행 횟수로 합산하지 않는다.

합성 모델은 센서 잡음·조향 지연·타이어 미끄러짐·차체 충돌과 도로 조건을 포함하지 않는다. 수치와 재현 명령은 [조향](ros1-port/docs/LATERAL_FOUNDATION_2026-10-25.md), [진행](ros1-port/docs/PROGRESS_RESULT_2026-10-25.md), [변환](ros1-port/docs/PATH_CONVERSION_RESULT_2026-10-25.md), [요청 관리](ros1-port/docs/PATH_SELECTION_RESULT_2026-10-25.md)의 각 결과를 따른다.

개발 규칙·작업 계획·단계별 전환 계획과 데이터 도구의 개발 문서는 CTRL 팀의 용어로 정리했다. 작업 목적·수정 범위·반영 절차·확인 방법·종료 조건을 공통 형식으로 사용하며 기존 명령·기술 값·검증 수치는 유지한다.

## 1단계에서 남은 작업

1. 현재 follower 입력으로 새 계산도 수행하는 병행 계산과 비교 기록 연결
2. 실제 센서 기록 재생과 Linux catkin/ROS 메시지·시각 확인
3. 차량 파라미터·조향 응답 확인 후 MORAI에서 일반/회피/local reference 및 전환 검증

현재 완료 범위는 **1단계의 오프라인 계산 기반**이다. 새 조향을 실제 제어 출력으로 전환하거나 1단계 전체를 완료한 상태는 아니다.

## 이 저장소의 두 코드 경로

| 위치 | 용도 |
| --- | --- |
| `ros1-port/` | 이번에 반영한 CTRL ROS1 실행 구성과 새 오프라인 제어 모듈 |
| `src/`, `docs/LOCAL_PLANNING.md`, `docs/UDP_CONNECTION.md` | SCMC_2026 main에 있던 기존 ROS1 프로젝트와 설명. 이번 반영에서 코드 내용 유지 |
| `2026_CTRL_competition/`, `과거자료/` | 비교 기준·이전 자료·문서 원문 보관 |

두 코드 경로를 같은 catkin workspace에서 함께 빌드하지 않는다. 이 README의 Docker 명령은 `ros1-port/`를 대상으로 한다. 기존 SCMC 프로젝트의 실행 구성은 [이전 README](docs/SCMC_PREVIOUS_README_2026-10-25.md), 가져온 범위와 이력은 [반영 기록](docs/CTRL_ROS1_IMPORT_2026-10-25.md)을 참고한다.

## 팀원이 먼저 볼 문서

- [현재 개발 계획](ros1-port/docs/DEVELOPMENT_WORKFLOW.md)
- [ROS1 설치·실행](ros1-port/README.md) · [Docker 환경](ros1-port/docker/README.md)
- [패키지·문서 전체 목차](ros1-port/docs/MARKDOWN_INDEX.md)
- [외부 참고 코드·작년 자료·출처 구분](ros1-port/docs/SOURCES_AND_HISTORY.md)
- [새 제어의 현재 결과와 1단계 남은 범위](ros1-port/docs/PATH_SELECTION_RESULT_2026-10-25.md)

## 실행

Linux에서 ROS1/Docker 환경과 CTRL 이름 변경 후 재빌드를 준비한다. 저장소 루트에서:

```bash
bash ros1-port/docker/run.sh bash docker/build_workspace.sh
```

호스트 명령 연결이 설치된 환경의 수신 점검:

```bash
roslaunch ctrl_start.launch drive:=false
```

주행 준비가 된 환경에서 사용하는 명령은 `roslaunch ctrl_start.launch`다. 이 launch의 기본 `drive`는 true이므로 제어를 송신할 수 있다. 설치·채널 연결·종료와 기록 위치는 [실행 안내](ros1-port/README.md)를 따른다. 이 문서 정리에서 명령을 실제 실행하지 않았다.

## 참고 코드와 기록

`2026_CTRL_competition/`은 FOSCAR의 2026 참고 원본을 보존한 비교 트리다. 폴더에 CTRL 이름이 있어도 CTRL 팀이 원본 전체를 작성했다는 뜻은 아니다. `과거자료/sim_2025/`는 2025 자료로 보관된 작년 참고 코드다. 두 자료의 연도·작성자·라이선스를 서로 바꾸지 않는다.

기존 MORAI 주행 결과는 [10/23 터널 기록](ros1-port/docs/TUNNEL_CONTINGENCY_2026-10-23.md), [사용자 주행 기록](ros1-port/docs/USER_FULLCOURSE_2026-10-23.md)에 당시 조건과 함께 남아 있다. 현재 재구현의 성공 근거와 구분해 읽는다.

## 이전 문서 원문

[교체 전 원문과 당시 기록](%EA%B3%BC%EA%B1%B0%EC%9E%90%EB%A3%8C/ctrl-document-history-20261025/ROOT_README.original.md). 원래 작성자·날짜·출처와 실험 수치를 보존했다.
