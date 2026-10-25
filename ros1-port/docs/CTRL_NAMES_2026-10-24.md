# CTRL 실행 이름 정리 — 2026-10-24

기준 커밋 `5fa6bd93503dcf25fae7d123541a3ebd08322302`에서 실행 패키지·자체 Python 모듈·실행 파일·launch 이름을 CTRL로 정리했다.
실행 코드는 `ros1-port/src/ctrl_*/`이며 비교 원본과 과거자료는 명명 변경 대상에서 제외했다.

## 이름 규칙과 범위

| 이전 | 현재 |
| --- | --- |
| `foscar_control` | `ctrl_control` |
| `foscar_planning` | `ctrl_planning` |
| `foscar_lidar` | `ctrl_lidar` |
| `foscar_camera` | `ctrl_camera` |
| `foscar_camera_train` | `ctrl_camera_train` |
| `foscar_bringup` | `ctrl_bringup` |
| `foscar_interfaces` | `ctrl_interfaces` |
| `foscar_ros1` | `ctrl_ros1` |
| `foscar_control/core.py` | `ctrl_control/ctrl_core.py` |
| `foscar_control/follower.py` | `ctrl_control/ctrl_follower.py` |
| `foscar_planning/behavior_node.py` | `ctrl_planning/ctrl_behavior_node.py` |
| `foscar_planning/states/yield_.py` | `ctrl_planning/states/ctrl_yield.py` |
| `scripts/path_follower` | `scripts/ctrl_path_follower` |
| `launch/competition.launch` | `launch/ctrl_competition.launch` |

패키지 내부 import·상태 동적 로더·catkin 설치 목록·XML launch·지원 도구의 경로도 함께 수정했다.
전체 대응표는 [ctrl_name_map.json](../tools/ctrl_name_map.json)이다.

모든 파일의 basename이 서로 달라지는 것은 아니다. 다음 이름은 의도적으로 유지한다.

- `package.xml`, `CMakeLists.txt`, `setup.py`, `__init__.py`: 도구가 요구하는 표준 이름.
- 테스트·upstream fixture의 파일명과 출처·라이선스: 기존 검증과 출처를 구분하기 위한 이름.
- 지도·모델·경로·설정 데이터의 파일명과 ROS 메시지 정의 파일명: 데이터·인터페이스 연결을 유지하는 이름.
- ROS 토픽·노드명·파라미터 키·메시지 필드·환경 변수: 기존 외부 계약.
- `2026_CTRL_competition/`, `과거자료/`, 날짜별 검증 JSON·manifest: 보존 자료.

따라서 명명 변경은 실행 코드의 팀 구분을 위한 것이다. 원본 작성자/유래를 바꾸거나 알고리즘을 새로 개발했다는 뜻이 아니다.

## 실행 전 한 번 재빌드

ROS 패키지·Python 모듈·실행 파일의 이름이 바뀌었으므로 이전 devel/install만 사용하지 말고 Linux의 기존 Docker 환경에서 재빌드한다.
저장소 루트에서:

```bash
bash ros1-port/docker/run.sh bash docker/build_workspace.sh
```

빌드 후 새 Bash 터미널에서 실행 진입점은 다음과 같다. 이 명령은 기본 주행을 시작한다.

```bash
roslaunch ctrl_start.launch
```

수신만 확인할 때는 `drive:=false`를 지정한다. 이번 Windows 작업에서는 Docker/catkin/ROS/MORAI를 실행하지 않았다.
메시지 필드와 바이트 정의는 같지만 생성 패키지는 `ctrl_interfaces`이므로 ROS 소비 프로그램도 새 빌드를 사용해야 한다.

## 검증 결과와 재현

- 런타임 Python 120개를 이름 역매핑한 AST가 기준 커밋과 동일하다.
- 전체 소스·실행 스크립트 262개의 AST와 launch 10개의 계약도 역매핑 후 동일하다. 모델·지도·경로·메시지 12개 파일은 바이트가 같다.
- 오프라인 알고리즘 검사: 변경 전후 각각 979개, 975 PASS / 같은 기존 실패 4개.
- 기존 실패: 삭제돼 있는 `measure_highway.py`·`replay_highway.py`를 요구하는 검사 3개와 ROUNDABOUT 지면 보정 기대값 충돌 1개. 기대값은 변경하지 않았다.
- 어댑터: 27개 실행, 26 PASS / 네이티브 환경 관련 1 SKIP.
- 패키징 단위 검사 8 PASS. 원본/CTRL 차등 검사 도구도 새 결과 디렉터리의 telemetry 2개씩을 읽어 양쪽 PASS를 확인했다.
- launch 패키징 계약: 인자 110개·노드 21개·파라미터 106개, bag 토픽 24개와 points 2개 검사.
- `tools/ctrl_resource_hashes.json`은 새 CTRL 배포 경로의 manifest다. 기존 `resource_hashes.json`은 보존했고 기존 불일치 2개도 새 manifest의 provenance에 기록했다.
- Bash/Noetic 의존 검사는 이 Windows 환경에서 완료하지 못했다. 실제 catkin 빌드·ROS 메시지 생성·제어·실주행 성공은 주장하지 않는다.

새 결과 디렉터리에 명명·AST·launch·리소스 검사를 실행한다:

```powershell
ros1-port/.python310/python.exe ros1-port/tools/run_refactor_checks.py --ctrl-names-only
```

알고리즘 전후 비교 원문은 작업 PC의 `C:/Users/rever/Coding/audit-reports/ctrl-names-20261024/`에 남겼다.
이 문서와 현재 실행 안내 외에 과거 실험 문서·검증 결과를 새로운 이름으로 덮어쓰지 않았다.
작업 브랜치는 `refactor/ctrl-names-20261024`이며 commit/push/main 병합은 실행하지 않았다.
