# 미사용 파일 정리 — 2026-10-22

## 목표·범위·완료 기준

현재 ROS1/MORAI 실행, 선택 기능, 빌드, 회귀 및 기록 재현에 필요한 파일을
구분하고 미사용이 입증된 파일만 삭제한다. 주행 로직과 공개 인터페이스는
변경하지 않는다. MORAI 실행, 제어 송신, commit/push/merge는 수행하지 않는다.

작업 브랜치는 `cleanup/unused-files-20261022`, 시작 HEAD는
`0fe5ae635f5acbc79aae9e9c7f88a3474a63b233`이다. 시작 브랜치는
`fix/static-course-20261021`이었다. 기존 수정 7개와 미추적 20개를 보존한다.

영향 파일은 삭제 목록에 명시한 과거 참고 트리의 파일과 이번 조사 도구·문서,
루트의 현재 안내 링크뿐이다. production, CMake, setup.py, launch,
YAML, 메시지, 모델 및 실제 사용 경로 파일은 변경하지 않는다.

구현 방법은 실행 루트에서 참조를 추적하고, 과거 사본의 설치·시험·재현
의존성을 별도로 조사한 뒤 삭제 전 원본을 보관하는 것이다. 검증 방법은
구문·import·패키징·native catkin 빌드/직렬화·launch 정적 해석·오프라인 회귀,
현재 문서 링크 및 전후 SHA256 비교다. 완료 기준은 삭제 파일별 근거·복구법,
전체 잔존 파일 분류, 실제 검사 결과와 미확인 범위를 함께 남기는 것이다.

## 삭제 전 복구 지점

Git 제외 폴더:
`ros1-port/verification/.work/unused-cleanup-20261022-120150/`.

- `before-source.tar.gz`: 추적 파일과 ignored가 아닌 미추적 파일 전체.
  **2,676개 / 312,221,708 bytes**의 정리 전 원문을 포함한다.
- `before-files.json`: 각 파일 SHA256·크기·심볼릭 링크 대상.
- `before.patch`, `before-status.txt`, `metadata.json`: 기존 사용자 변경과 Git 기준.
- tar.gz SHA256:
  `10e305a9a4f64f90a8c32d248ee18915334dcbab51012e08cfca794d8ebd8c39`.

`static_course.py`, 새 회귀 검사, ground/avoid 수정과 시험 감독 도구도 포함한다.
build/devel/install, bag·영상·artifacts 등 ignored 데이터는 이 소스 백업에
포함하지 않으며 원래 위치에 보존한다. 백업이 있으므로 ignored 데이터를
지워도 된다는 의미가 아니다.

## 결과

삭제 4개 완료. 아래에 참조 분석, 보류 사유, 전후 검증과 복구 방법을 기록한다.

## 실제 삭제 목록

삭제 파일은 총 **4개 / 1,199,942 bytes (1.144 MiB)**다. 각 파일은 삭제 전
백업에서 SHA256을 다시 확인했다. 대체 파일은 동일한 역할의 실제 빌드/경로
파일이며, 사용하지 않던 루트 헤더와 src 헤더가 바이트 동일하다는 뜻은 아니다.
설치 목록에 삭제 대상이 없으므로 CMake/setup 수정은 필요하지 않았다.

| 삭제 경로 | 역할·미사용 근거 | 남긴 파일 | 복구 |
| --- | --- | --- | --- |
| `sim_2025/Simulator_2025-ai_mission/src/lidar/header.h` | 모든 C++ 소스가 같은 src 디렉터리의 ./header.h를 include. CMake의 include/export/install도 src만 사용하며 루트 헤더 참조 및 동적 glob 없음. | `sim_2025/Simulator_2025-ai_mission/src/lidar/src/header.h` | 백업의 같은 경로 또는 아래 git restore |
| `sim_2025/Simulator_2025-main/Simulator_2025-main/src/lidar/header.h` | 모든 C++ 소스가 같은 src 디렉터리의 ./header.h를 include. CMake의 include/export/install도 src만 사용하며 루트 헤더 참조 및 동적 glob 없음. | `sim_2025/Simulator_2025-main/Simulator_2025-main/src/lidar/src/header.h` | 백업의 같은 경로 또는 아래 git restore |
| `sim_2025/Simulator_2025-ai_mission/src/planning/scripts/make_path/main_interpolation.txt` | paths/의 실제 경로와 SHA256 동일. planner·설치 목록에서 make_path 하위 txt 참조가 없고 폴더 glob 소비자 없음. 해당 폴더에 실행 가능한 편집기 없음. 독립적인 예전 편집기 버전이 든 ZIP은 고유 코드가 있어 보존하며, ZIP에도 같은 경로 데이터가 포함됨. | `sim_2025/Simulator_2025-ai_mission/src/planning/paths/main_interpolation.txt` | 백업의 같은 경로 또는 아래 git restore |
| `sim_2025/Simulator_2025-main/Simulator_2025-main/src/planning/scripts/make_path/zzinmak.txt` | paths/의 실제 경로와 SHA256 동일. planner는 paths/만 읽으며 CMake는 PathPlanner.py만 설치. 편집기의 현재 진입점은 home.txt를 읽고 삭제 후보를 열지 않음. 폴더 glob 소비자 없음. | `sim_2025/Simulator_2025-main/Simulator_2025-main/src/planning/paths/zzinmak.txt` | 백업의 같은 경로 또는 아래 git restore |

파일별 SHA256·크기·삭제 이유는 [삭제 manifest](cleanup-2026-10-22/deletions.json)에 남겼다.

## 보존·보류와 판단 근거

아래 판단은 이름이나 팀명에 따른 일괄 보존이 아니라 실제 비교 결과다.
현재 실행·검증은 ROS1 트리와 competition 원본을 사용한다. 다른 사본이 현재
주행에 쓰이지 않는다는 사실과, 그 사본의 고유 시험·과거 입력을 재현하는 데
필요하지 않다는 판단은 구분했다. 안전하게 삭제할 근거가 부족한 부분은 남겼다.

| 대상 | 조사 결과와 보존/보류 이유 |
| --- | --- |
| `2026foscar/` | 367개: 원본과 동일 275, 내용 차이 76, 원본에 없는 파일 16. integration_udp.py가 config/route를 하드코딩하고 측정 wrapper가 run_low_speed.sh를 호출한다. 독립 시나리오·random_ped.py도 있음. |
| `2026foscar-team2/` | 272개: 원본과 동일 221, 내용 차이 44, 원본에 없는 파일 7. 고속도로 last_change/slot_why 고유 테스트·보고서 및 당시 설정/정책이 있음. 시험·설치가 자기 트리 모듈/모델을 참조하므로 동일 파일만 빼면 재현 트리가 깨짐. |
| `2026foscar-records/` | 434개: 원본과 동일 401, 내용 차이 33, 원본에 없는 파일 0. protected 원본의 records_readme.py가 이 폴더 README를 갱신 대상으로 참조한다. README·pass_records는 기록 보존, 다른 소스 차이는 제어/인지/계획 당시 버전이다. 재현 의존성을 끊지 않고 일부만 삭제할 근거 부족. |
| `2026foscar-competition/` | 명시적 보존 원본. 현재 차등 검사·packaging·원본 tools를 통한 고속도로 시험의 비교 기준. 모든 바이트 보존 |
| `ros1-port/src/` 전체 | entrypoint, importlib 상태, 설치 목록, 선택/대체 경로, 회귀 검사가 연결되어 있다. 최근 미추적 static_course.py 포함 전부 보존 |
| `*.launch.py`, camera_train, GPS logger, fake_cloud, monitor/view, BEV/HUD | ROS1 비교 검사 또는 제공되는 학습·수집·시각화 기능. 기본 launch에서 꺼진 이유만으로 삭제하지 않음 |
| `sim_2025/` 나머지 | 두 트리는 control/lidar/planning 정책이 다르고 외부 자료 출처다. morai_msgs 사본 113개는 동일하지만 각 옛 catkin 워크스페이스의 빌드 의존성이다. ZIP의 path_editor.py도 고유 코드여서 압축 사본으로 단정하지 않음 |
| `tunnel_tools/`, `측정테스트파일/` | 과거 bag/실험·측정의 재현 도구, 하드코딩 외부 경로/ROS2 설치가 있음. 기록에서 호출되는 분석 도구를 보존. 동적 외부 입력의 현 사용 여부는 확인 불가 |
| 루트 global_path.txt / global_path_smoothed.txt, 제공 K-CITY 지도·경로 | 현재 launch 경로와 다르지만 10/16 경로 차이 분석의 입력·공식 자료 비교 근거. 파일명 중복으로 삭제하지 않음 |
| 날짜별 문서, JSON, SHA256 manifest, bag·영상·충돌·통과 기록 | 과거 결과 원문을 보존. 루트 현재 README에는 새 안내 링크만 추가 |
| build/devel/install | 기존 Docker entrypoint가 devel/setup.bash를 source한다. build는 CMake 경로/생성 메시지 캐시, install은 설치 실행 검증 자원이다. 재생성은 docker/build_workspace.sh 및 catkin_make install이지만 현재 실행 의존성을 고려하여 삭제하지 않음 |
| docker/artifacts | 4,093개 / 8,573,302,554 bytes의 원본 실험·센서·주행 기록. 용량 확보 대상으로 삭제하지 않음 |

라이선스·저작권·출처·작성자 문서와 package/setup의 선언은 전부 보존했다.
삭제한 네 파일에 별도의 저작권/작성자 선언은 없으며 원문도 Git/백업에 있다.
CSV의 개발·검증 분류에는 이러한 보존 원본·과거 자료가 포함된다.

## 전체 파일 분류와 실행

[파일·미션 안내](FILE_MISSIONS_2026-10-22.md)에 실행 흐름·미션별 주요 파일·
실행 명령을 정리하고, [전체 잔존 파일 CSV](cleanup-2026-10-22/retained-files.csv)에
각 파일의 공통/신호/정적/보행자/회전교차로/고속도로/터널/개발·검증 분류와
공유 미션·보존 역할을 기재했다. 목록은 ignored 소스 외 데이터까지 runtime으로
간주하지 않는다. ignored 데이터는 아래 별도 전후 검사를 따른다.

## 검증 결과와 한계

검사 원문은 백업 폴더의 `checks/` 아래에 새로 저장했다.
[검사 요약 JSON](cleanup-2026-10-22/test-summary.json)에도 종료코드와 범위를
남겼다. 검사 기대값·원본 모델·기존 manifest를 변경하지 않았다.

| 검사 | 이번 결과와 해석 |
| --- | --- |
| 전체 Python 구문 | 1,426개 compile PASS; 삭제 전 1,425개도 PASS. 추가 1개는 이번 조사 도구. pyc 생성 없이 확인 |
| 명시적 native entrypoint import | bridge/localization/follower/behavior/static_course/lidar/camera 9개 PASS. main/노드 생성 호출 없음 |
| catkin build / install | 격리한 native 워크스페이스에서 모두 PASS. 원래 build/devel/install에 쓰지 않음 |
| native 메시지·TF·cloud 직렬화 | PASS. ControlCommand/VehicleStatus/Event/DrivingTarget·Marker, big/little endian cloud·TF lookup |
| ROS1 adapter | 27 PASS, 0 skip |
| 알고리즘 회귀 | control/planning/lidar/camera 927 PASS, 0 FAIL, 0 skip; trace 35,609개. ROS 공통 대역을 사용하며 실제 전송·주행 시험 아님 |
| 재생·시험 감독 도구 | ground/sequence replay와 course_audit_safety 합계11 PASS. 위 927개와 중복 집계하지 않음 |
| launch 정적 해석 | 기본 local.launch의 --nodes / --files PASS. 노드·master를 실행하지 않음. 지연 자식 launch는 XML/보조 스크립트도 별도로 추적 |
| packaging 단위 | 7 PASS / 1 ERROR. 최근 incoming behavior.yaml이 동결 원본 리소스 크기와 달라 실패. 이 파일은 이번 작업에서 변경하지 않음 |
| packaging 정적 | FAIL 유지. 삭제 전에도 DrivingTarget.msg에서 실패했고, 메시지4개 모두 원본 LF/이식본 CRLF의 바이트 차이 확인. 줄바꿈 정규화 비교는 모두 동일. 검사 순서상 첫 보고 파일은 달라질 수 있음 |
| devel/install 환경 검사 | 각각9 PASS / 1 resource FAIL / 1 KNOWN_SOURCE_ISSUE. resource는 incoming behavior.yaml 크기 불일치, known은 기존 best.onnx README SHA 불일치. 전체 PASS로 보고하지 않음 |
| identifier AST / boundary | FAIL 유지. 이미 존재하던 ground/avoid 기능 변경은 예전 지역변수 정리만 허용하는 기준과 다름. 원본/기대값은 그대로 |
| preflight | FAIL 유지. 위 boundary/packaging 차이와 기존 모델 README SHA 불일치. 삭제로 바뀐 production 파일 없음 |
| 현재 안내 링크 | 검사 대상9문서의 로컬 파일 링크 전부 존재. 아래 재현 경로의 current-links.json 참조. 과거 문서의 기존 링크 불일치는 수정하지 않음 |
| 원본·사용자 변경 보존 | 정리 전 manifest와 비교해 README 안내 링크 외 기존 파일 수정0, 허용 목록 외 삭제0. static_course 등 미추적 코드도 바이트 동일 |
| ignored 데이터 | 기존 build 1,008개 / devel 103개 / install 276개 / artifacts 4,093개의 lstat 크기·mtime_ns·symlink 대상 동일. 컨테이너 절대경로의 기존 dangling symlink도 포함. 8GB 데이터 전부를 새로 SHA 재검증한 결과와는 구분 |
| whitespace | git diff --check는 기존 incoming CRLF 때문에 실패. core.whitespace=cr-at-eol 검사 PASS. 이번 삭제·새 문서 때문에 생긴 whitespace 오류 없음 |

10/21의 기존 932개 검사 기록은 알고리즘927개와 감독 도구5개를 합친 결과다.
당시931 PASS/1 FAIL은 고속도로 tick 평균 시간 기준 실패였고 원문을 보존했다.
이번에는 해당 알고리즘 검사도 PASS였으나 타이밍 안정성을 고쳤다는 뜻은 아니다.
새 재생 검사6개를 더해 위의 도구 검사11개가 되었다.

MORAI/drive=true/제어 UDP 송신, 실제 ROS node·master 시작, ROS2 middleware,
옛 sim_2025 C++ 전체 빌드 및 Windows Python 실행은 수행하지 않았다.
C++ 삭제의 검증은 include의 상대경로 해석과 CMake export/install 목록 검사다.
삭제 후보 밖에 있는 고유 ZIP 편집기, 외부 경로를 사용하는 측정/재현 도구의
호환성은 미확인으로 남겼다. 전체 코스 완주 여부에 새 근거를 추가하지 않는다.

## 재현 명령

다음 조사는 파일을 지우지 않는 read-only 도구다. 출력은 매번 새 폴더를 쓴다.

```bash
cd /home/foscar/RMT-ros1
rmt_cleanup_check="ros1-port/verification/.work/cleanup-review-$(date +%Y%m%d-%H%M%S)"
python3 ros1-port/tools/audit_unused_cleanup.py \
  --baseline ros1-port/verification/.work/unused-cleanup-20261022-120150/before-files.json \
  --output "$rmt_cleanup_check"
git -c core.whitespace=cr-at-eol diff --check
```

이번 native/회귀 실행 스크립트도 백업 폴더의 `verify.py`에 보관했다.
새 결과 폴더를 만들어 아래처럼 재실행할 수 있다. 이 스크립트는 catkin 복사본과
오프라인 검사만 시작하며, 모든 roslaunch 호출에 --nodes/--files를 지정한다.
packaging 등 기존 실패는 checks/results.json에 개별 종료코드로 남는다.
스크립트 자체 종료코드0을 모든 검사 성공으로 해석하지 않는다.

```bash
rmt_cleanup_repeat="verification/.work/cleanup-repeat-$(date +%Y%m%d-%H%M%S)"
mkdir -p "ros1-port/$rmt_cleanup_repeat"
bash ros1-port/docker/run.sh python3 \
  verification/.work/unused-cleanup-20261022-120150/verify.py \
  "/workspace/ros1-port/$rmt_cleanup_repeat"
```

원본·기존 결과를 수정하는 check_preservation/run_differential의 직접 실행이나
port_boundary --write는 하지 않았다. 원본과 현재 소스의 함수 동등성을 다시
주장하는 차등 시험 대신, 이번 삭제 전후 runtime 전체 SHA256 동일과 기존
수정본 회귀를 확인했다. 실제 필요한 기능 수정은 이번 작업에 섞지 않았다.

## 복구 방법

네 삭제 파일은 원래 clean 추적 파일이므로 다음 명령으로 **네 파일만**
기준 HEAD에서 복원할 수 있다. 기존 수정·미추적 파일은 건드리지 않는다.

```bash
git restore --source=0fe5ae635f5acbc79aae9e9c7f88a3474a63b233 -- \
  sim_2025/Simulator_2025-ai_mission/src/lidar/header.h \
  sim_2025/Simulator_2025-main/Simulator_2025-main/src/lidar/header.h \
  sim_2025/Simulator_2025-ai_mission/src/planning/scripts/make_path/main_interpolation.txt \
  sim_2025/Simulator_2025-main/Simulator_2025-main/src/planning/scripts/make_path/zzinmak.txt
```

미추적 static_course.py 등 Git HEAD에 없는 사용자 작업은 tar에서 복구한다.
먼저 별도 디렉터리에 꺼내 확인한 뒤 필요한 파일만 작업 폴더에 복사한다.
다음 명령은 작업 중인 소스에 덮어쓰지 않는다.

```bash
mkdir -p /tmp/rmt-cleanup-restore-20261022
tar -xzf ros1-port/verification/.work/unused-cleanup-20261022-120150/before-source.tar.gz \
  -C /tmp/rmt-cleanup-restore-20261022
```

Git clean/reset/commit/push/merge는 실행하지 않았다. 이 백업은 로컬 ignored
데이터이므로 다른 PC로 옮길 때는 사용자가 별도로 보관해야 한다.

## 삭제 전후 파일 수·용량

<!-- CLEANUP_METRICS_BEGIN -->
| 집계 범위 | 정리 전 | 정리 후 |
| --- | ---: | ---: |
| 추적 + ignored 아닌 미추적 파일 | 2,676개 | 2,678개 |
| 위 파일의 논리 크기(lstat) | 312,221,708 bytes | 311,456,804 bytes |
| 시작 파일 중 보존된 파일 | 2,676개 | 2,672개 |

삭제 4개에서 1,199,942 bytes를 줄였고, 새 조사 도구·보고·분류 자료6개와
README 링크를 추가했다. 보고 자료를 포함한 순감소는 764,904 bytes다.
Git/object·ignored 환경과 bag/artifacts·새 로컬 백업/빌드 사본은 소스 집계에서
제외한다. 로컬 전체 디스크 사용량은 복구 tar와 검증 사본을 추가했으므로
이 수치만큼 줄었다고 주장하지 않는다.

기존 build/devel/install의 논리 크기는 각각2,782,074 / 395,563 /
52,371,051 bytes이며 삭제0이다. 기존 artifacts는8,573,302,554 bytes,
삭제0이다. 실제 디스크 할당 du와 심볼릭 링크/파일시스템 block 기준은 다르다.
소스 전후 전체 목록·해시는 before-files.json 및 이번 audit-final/ 아래에,
ignored 파일별 원래 목록은 build-before.json, devel-before.json,
install-before.json, docker-artifacts-before.json에 별도로 보존했다.
<!-- CLEANUP_METRICS_END -->
