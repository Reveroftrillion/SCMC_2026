# 대회용 ctrl_start.launch — 2026-10-23

## 개발 목표·영향 파일

사용자가 요청한 `roslaunch ctrl_start.launch`를 현재 Docker 작업 디렉터리에서
실행할 수 있게 한다. 기존 run_course.sh와 같은 시작점 초기화·통신 포트·노드·bag
구성을 사용하며 주행 알고리즘과 기존 launch의 인자·기본값은 변경하지 않는다.
과거 자료 통합 후속 작업이며 브랜치는 `cleanup/archive-legacy-20261023`이다.
사용자 기존 변경·미추적 파일을 보존했다.

- `ctrl_start.launch`: 현재 호스트용 대회 진입점. drive/bag/raw 기본 true.
- `docker/run_course.sh`: 동적으로 manual.launch를 만드는 대신 공통 진입점 사용.
  기존 무인자 drive=false·console 필터·실행별 로그 경로 유지.
- `src/foscar_bringup/scripts/launch_info`: prepare-run-dir 명령 추가.
  노드 실행 전 기록 디렉터리를 준비하며 UDP·ROS 노드를 실행하지 않는다.
- 루트/ROS1 README: 컨테이너 준비·한 번 실행·종료·기록 경로 안내.
- `tools/test_ctrl_start.py`: 노드 기동 없는 launch 해석·기존 래퍼 경계 검사.

## 구현 방법·완료 기준

local.launch → competition.launch를 포함해 현재 포트를 그대로 사용한다.
START 첫 랩 파라미터3개, follower 로그 경로, 원본 센서 기록기를 정적 launch로 옮겼다.
직접 launch는 익명 실행별 디렉터리를 기본값으로 사용한다. 래퍼는 기존 mktemp
디렉터리를 run_dir 인자로 전달한다. 직접 launch의 console 필터와 ROS 로그 경로는
래퍼와 구분하며 README에 명시했다.

현재 Docker에서 catkin 환경을 로드한 뒤 roslaunch ctrl_start.launch로 전체 구성이
해석되고, drive=false 및 기록 토글·래퍼 기본값이 보존되면 구성 변경을 완료한다.
실주행 성공은 별도 검증 대상이다. 현재 코드에서 설치 대상인 launch_info의 새 명령이
필요하므로 별도 install 공간 배포 시 재빌드·재설치해야 한다. 현 Docker devel 실행의
패키지 조회는 소스 디렉터리의 scripts/launch_info를 사용한다.

## 검증·재현

```bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 ros1-port/tools/test_ctrl_start.py
bash -n ros1-port/docker/run_course.sh
git diff --check
```

3 tests PASS. 실제 ROS1 XmlLoader로 구성만 해석했고 노드·master·UDP 송신은 실행하지 않았다.
기본 drive=true, 수신전용 false, 첫 랩3개, Status909/GPS1111/IMU1112/제어9093,
LiDAR2368·지연 카메라9297, 두 기록기·bag/raw 토글·공백 포함 기록 경로를 확인했다.
래퍼는 임시 사본과 가짜 run.sh로 실행해 drive 기본값·명시 인자 전달을 확인했다.
구문·문서 링크·diff whitespace 검사도 수행한다.

Docker 컨테이너 기동·catkin/native 빌드·Windows·MORAI 실제 수신/주행/종료·GPU 추론은
이번에 실행하지 않았다. 과거 주행 결과를 새 launch의 실주행 성공으로 주장하지 않는다.
커밋·push·merge는 하지 않았다.
