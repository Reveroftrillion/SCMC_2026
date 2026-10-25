# 호스트 roslaunch와 후속 개발 — 2026-10-23

## 목표·범위·완료 기준

사용자가 최신 main을 직접 실행해 다시 완주했다고 보고했고, 호스트에서
`roslaunch ctrl_start.launch` 한 명령으로 실행하도록 요청했다.
작업 브랜치는 `feat/host-roslaunch-20261023`이다.

- 목표: Docker 터미널 진입을 없애고 기존 검증 환경과 인자·종료 신호를 유지한다.
- 영향 파일: docker/host_setup.bash, tools/install_host_roslaunch.py와 관련 검사,
  현재 README·계획·이 문서. 기록 파일명 오류는 별도 재현 후 launch_info만 수정한다.
- 방법: Bash에서 정확히 첫 인자가 ctrl_start.launch인 명령만 기존 run.sh로 연결한다.
  나머지 roslaunch 명령은 원래 실행 파일로 전달한다. Python3.8 호스트에 Docker의
  Python3.10 catkin overlay를 source하지 않는다. 시스템 ROS 파일·일반 launch·제어 정책은 보존한다.
- 검증: 가짜 Docker/roslaunch로 인자·공백 경로·종료 코드·SIGINT·다른 명령 전달,
  설치 백업·재설치·손상된 설정 거부를 검사한다. 기존 launch는 XML만 해석한다.
- 완료 기준: 현재 사용자 Bash 설정에 백업 후 등록, 새 터미널에서 명령 인식 확인,
  실제 제어를 시작하지 않고 정적/모의 검증 결과와 남은 실제 확인을 기록한다.

## 사용

이 PC에는 설치 도구로 `~/.bashrc` 마지막에 작은 source 블록을 추가한다.
다른 PC에서 한 번 설치할 때:

```bash
cd /home/foscar/RMT-ros1
python3 ros1-port/tools/install_host_roslaunch.py
source ros1-port/docker/host_setup.bash
```

이후 새 Bash 터미널에서는 어느 디렉터리에서든 다음 명령을 사용할 수 있다.

```bash
roslaunch ctrl_start.launch drive:=false
# 수신 점검을 종료하고 MORAI 시작점·연결을 준비한 뒤 주행
roslaunch ctrl_start.launch
```

기본 drive=true·bag=true·raw=true와 기존 첫 랩·포트·기록 위치를 유지한다.
내부에서는 기존 Docker가 실행되며 기존 이미지와 빌드가 필요하다.
인자는 그대로 전달한다. run_dir는 컨테이너의 /workspace 아래 경로를 사용한다.
Ctrl+C는 기존 Docker 경로로 전달된다. 실제 정지·Manual/P 인계 확인은 계속 필요하다.
`roslaunch 다른패키지 다른.launch`와 `roslaunch --help`는 호스트 ROS로 전달한다.
`roslaunch` 앞에 옵션을 붙여 CTRL을 호출하는 형식은 이 연결 대상이 아니다.

현재 열려 있던 사용자 터미널에는 위 source 명령을 한 번 실행하거나 새 터미널을 연다.
설치 해제는 ~/.bashrc의 CTRL ROS1 launch 블록을 제거하고 새 터미널을 열면 된다.
기존 설정은 .bashrc.ctrl-backup-날짜시각에 보존하며 중복 설치는 블록을 늘리지 않는다.

## 기록 파일명 개행 수정과 검사

최신 bag에도 이름이 `run_날짜_시각\n.bag`인 파일을 확인했다.
ROS1의 command 파라미터는 stdout의 개행을 보존하는데 launch_info의 print가
항상 개행을 붙여 bag_name·run_dir·code_version에 섞였다.
실제 launch 해석과 recorder의 exec 인자를 검사해 수정 전2 FAIL을 재현했다.
launch_info의 stdout만 개행 없이 출력하도록 수정했다. 토픽·인자·기록기는 유지하며
과거 bag을 이동·이름 변경하지 않는다. 최종 실제 Docker 기록 생성은 별도 확인 대상이다.

## 카메라 종료 오류 후속 개발

- 목표: 정상 외부 종료 때만 traceback을 없애고 예상하지 못한 오류는 계속 보고한다.
- 영향 파일: camera_bridge.py·yolo_signal_node.py의 main 예외 처리와 도구 검사.
- 원인: 실제 ROS1 adapter executor는 rospy 종료를 관측하면 ExternalShutdownException을
  발생시키지만 두 main은 KeyboardInterrupt만 처리한다. finally 자원 정리 후 예외가 남는다.
- 재현·검증: 실제 main과 실제 adapter 예외 클래스를 사용하고 init·spin·노드 생성만 대역 처리한다.
  소켓·GPU 모델·master·UDP를 만들지 않는다. 정상 종료·일반 오류·정리 오류를 구분한다.
- 완료 기준: 정상 종료는 자원 정리 후 반환, 일반 오류와 정리 오류는 원래대로 전파.
  실제 카메라 프로세스의 SIGINT 종료와 인계까지 이번 오프라인 검사로 통과 주장하지 않는다.

## 사용자 완주와 기록 범위

사용자가 최신 main으로 실행 후 완주했다고 보고했다. 별도 감독 시험과 합산하지 않는다.
최신 artifacts/ctrl_start_foscar_44_8710809407992363305의 follower CSV에서
3,563행·첫 FINISH elapsed173.550초·FINISH93행·마지막 속도0.00000119m/s를 확인했다.
기록 경과178.150초는 공식 완주 시간이 아니다. 최고 follower 속도는116.463km/h다.
원본 충돌·공식 차선·감점·Manual/P 인계·수정 소스의 주행 당시 고정 여부는
이 CSV만으로 확정하지 않는다. run-04 등 과거 실험 결과는 수정하지 않는다.

## 다음 개발 순서

1. 이번에 호스트 명령 등록과 bag 이름 개행 오류 재현·수정·오프라인 검사를 완료했다.
   다음 수신전용 실행에서 실제 새 파일명과 종료 신호 전달을 확인한다.
2. 카메라 정상 외부 종료의 최소 수정·오류 전파·자원 정리 검사를 완료했다.
   다음 실제 종료에서 두 노드의 traceback·exit 상태와 Manual/P를 확인한다.
3. 점수 검증: 제공 규정집의 차선 접촉·신호·충돌 판정과 실제 결과를 대조한다.
   사용자 완주와 follower FINISH는 공식 점수 통과를 뜻하지 않는다.
4. 속도: 규정집은 일반 구간60kph, A2256W000411 시작~A2256W000153 끝 예외를 명시한다.
   예외 경계를 실제 궤적과 대조한 뒤 구간별 Status 속도를 평가한다. 최고116km/h만으로
   과속 판정을 내리거나 속도 정책을 바꾸지 않는다.
5. 다른 배치·출현 시점·가림·입력 손실/재연결·부하를 한 조건씩 검증한다.
6. 설치 배포·원본/리소스/식별자 검사 정책을 정리한다. 과거 해시 기대값은 보존한다.

근거 규정은 [보존 규정집](../../과거자료/규정집_v1.1.5_예비_대학생AISW모빌리티%20경진대회.md)의
속도 제한·차로 준수 항목이다. 현재 대회가 같은 버전을 쓰는지는 별도 확인 대상이다.

## 작업 결과·재현

- ~/.bashrc 등록 완료. 원문은 /home/foscar/.bashrc.ctrl-backup-20261023-144646-287356에 보존했다.
  실제 새 Bash가 roslaunch를 function으로 인식하는 것을 확인했다.
  검사 터미널은 TTY가 없다는 경고와 sandbox의 .ros 캐시 쓰기 제한을 출력했으며
  함수 등록 확인은 성공했다. 사용자 대화형 터미널의 실제 Docker 기동은 별도다.
- 호스트 Python3.8에서 명령 연결7·기존 launch3·메타데이터2 PASS.
- Docker Python3.10에서 아래4개 도구를 함께 실행해15 PASS, FAIL/error/skip0.
  호스트 검사와 중복되므로 합산하지 않는다.
- 최종 Python7개 구문·JSON1개·현재 문서 내부 링크235개·git diff --check 통과.
  비교 원본530개 Git blob 불변, ~/.bashrc의 기존 원문과 두 미추적 빈 파일도 보존했다.
- 카메라 수정 전 정상 외부 종료의2개 subcase에서 ERROR를 재현했고 수정 후 통과했다.
  KeyboardInterrupt·일반 오류·정리 오류의 처리/전파도 확인했다.
- 초기 SIGINT 검사1개는 Python3.8/3.10의 enum 문자열 표시 차이로 실패했다.
  신호 번호 정수2를 비교하도록 신규 검사만 바로잡았고 두 환경에서 통과했다.
  기존 시험 기대값·검증 JSON은 수정하지 않았다.
- 실제 주행·센서·제어·전체 알고리즘950개·Windows·catkin 재빌드/설치는 이번 작업에서
  실행하지 않았다. 앞선950 PASS를 이번 변경 전체의 새 검사 결과로 합산하지 않는다.
- 이번 작업에서 커밋·push·main 병합은 하지 않았다. 비교 원본·과거 자료와 기존 빈 파일은 보존한다.

```bash
# 호스트: 가짜 Docker를 사용한 연결·설치·SIGINT 검사
PYTHONDONTWRITEBYTECODE=1 python3 ros1-port/tools/test_host_roslaunch.py

# 실제 사용 Python3.10에서 전체 도구 검사. 모델·소켓·ROS master·UDP 시작 없음
bash ros1-port/docker/run.sh env PYTHONDONTWRITEBYTECODE=1 \
  python3 -m unittest tools.test_host_roslaunch tools.test_ctrl_start \
  tools.test_launch_metadata tools.test_camera_shutdown
```

소규모 변경은 launch 메타데이터와 카메라 main의 예외 처리뿐이다.
주행 임계값·ROI·기억·속도·경로·executor 동작·ROS 인터페이스는 바꾸지 않았다.
[공유 검사·사용자 기록 요약](host-launch-20261023/summary.json)을 함께 보존한다.
