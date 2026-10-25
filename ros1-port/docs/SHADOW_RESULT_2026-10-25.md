# CTRL 병행 계산·비교 기록 연결

## 목적과 현재 상태

현재 follower 입력으로 새 경로 선택·진행 관리·조향 계산을 수행하고 기존 조향과의 차이를 기록한다. `shadow_config_file` 기본값은 빈 문자열이다. 설정 파일을 지정한 경우에만 기록 기능이 시작되며, 새 계산 결과를 `ControlCommand`에 넣는 경로는 없다. 1단계의 계산 기반과 비교 기록 연결까지 구현했고 Linux·실제 입력·MORAI 확인은 남아 있다.

## 연결 계약

- 기존 명령·상태·telemetry·표시 경로·CSV를 처리한 뒤 입력을 복사한다. 비교 기준은 테스트용 steering kick까지 반영한 실제 publish 조향각이다. 실제 UDP 전달이나 차량의 수신을 확인하는 기능은 아니다.
- raw ROS 패킷 대신 기존 follower가 수락한 `Target`을 복사한다. `INVALID_TARGET`/`CLOCK_RESET`은 유효한 비교로 취급하지 않는다. 따라서 새 수신기나 독립된 위치 추정기를 검증한 결과가 아니다.
- 위치는 odometry의 stamp/frame과 기존 follower가 계산에 사용한 x/y/yaw/speed다. 경로는 현재 route의 좌표와 `-controller.s_offset`을 전역 거리 원점으로 사용한다. 최초 획득·경로 revision 변경 시 기존 `output.progress` 주변의 명시적 범위에서 새 진행을 획득한다. 초기 구간 선택에는 기존 제어기에 대한 의존성이 남아 있다.
- 기준선·preview·chain·local reference는 새 선택기의 계약을 따른다. 진입 위치가 없는 legacy offset은 요청 수락 시 현재 진행 위치와 속도로 anchor를 만들고, 같은 요청의 heartbeat에서는 anchor를 유지한다. 전환 길이는 설정한 최소 길이와 `abs(speed) * transition_time` 중 큰 값이다. 0 복귀 요청에도 고정 anchor를 사용한다. 이는 비교용 연결 규칙이며 기존 제어기의 내부 궤적을 복제한다는 뜻은 아니다.
- `CRUISE`에서만 조향 차이를 계산한다. 속도 상한·정지 위치는 선택 metadata로 유지하지만 이 모듈에서 새 종방향 제어를 수행하지 않는다. invalid 계산에는 조향 후보가 없다. 긴 입력 공백·시계 되감기 뒤의 자동 재획득은 제공하지 않으며 필요 시 observer를 재시작하고 기록을 분리한다.

## 기록과 오류 처리

기존 CSV 옆에 같은 이름의 `.shadow.jsonl`을 독점 생성한다. header에는 경로와 계산 설정, frame에는 입력·유효 여부·거절 사유·기존/신규 조향·차이·계산 시간·누적 drop 수를 기록한다. NaN/Inf는 표준 JSON에서 허용되는 명시적 tag로 보존한다. 파일을 덮어쓰지 않는다.

worker가 계산과 frame 기록을 처리하고 대기 frame은 최대 1개다. worker가 밀리면 오래된 대기 입력을 버리고 최신 입력을 남긴다. `dropped`가 있으면 기록을 모든 tick의 연속 추적으로 해석하지 않는다. 설정/초기 경로/파일 생성은 처음 활성화되는 callback에서 수행하고, snapshot 복사와 Python worker도 CPU를 사용한다. 기존 명령값 유지 검사는 실행 시간 영향이 없다는 보장이 아니다.

설정·파일·worker 오류는 observer를 비활성화하고 로그에 남기며 기존 제어 계산을 계속한다. 종료 시 최대 1초 기다리며 기록을 마치지 못하면 불완전 로그를 알린다. 설정을 고친 뒤에는 노드를 재시작한다. 저장 실패 시 기록이 없거나 일부만 남을 수 있다.

## 재현 방법

저장소 루트에서 Python 환경이 있는 경우:

```powershell
& .\ros1-port\.python310\python.exe -X utf8 -B .\ros1-port\tools\validate_shadow.py
```

현재 Windows 게시 worktree에서는 기존 환경의 절대 경로를 사용했다:

```powershell
& C:/Users/rever/Coding/RMT-ros1/ros1-port/.python310/python.exe -X utf8 -B ros1-port/tools/validate_shadow.py
```

매번 새 `ros1-port/verification/.work/shadow-*` 폴더에 결과를 생성한다. 기존 기록은 아래처럼 ROS 없이 재생한다:

```bash
python3 -B ros1-port/tools/validate_shadow.py --replay /path/to/follower.shadow.jsonl
```

합성 검사 설정을 새 파일로 내보내는 명령:

```bash
python3 -B ros1-port/tools/validate_shadow.py --example-config /tmp/ctrl-shadow-example.json
```

이 파일의 차축 길이·조향 제한·시각 제한·전환 길이는 합성 검사 조건이다. 실제 차량의 보정값으로 간주하지 않는다. 검토한 설정을 follower의 private parameter `shadow_config_file`에 지정하고 노드를 시작하면 비교 기록을 활성화할 수 있다. 기존 launch에 활성화 인자를 추가하거나 실제 노드를 구동하지 않았다.

## 확인 결과

- Windows Python 3.10: **370 PASS**, 실패·오류·skip 0. 새 조향/진행/변환/선택 검사 278개, 신규 observer/연결 검사 24개, 기존 core·reference wire·route start 회귀 68개다.
- 기존 라이브러리의 Pyparsing deprecation warning 10건이 있었다.
- 일반·legacy offset·local reference 각 200프레임, 총 **600프레임**에서 후보 계산이 유효했고 저장/재생 결과가 일치했다. 고정된 합성 pose를 입력한 결정성 검사이며 폐루프 차량 성능 검사가 아니다.
- recorder worker의 bounded queue/drop, 파일 충돌, worker 오류, 재생 변조 검출을 확인했다. ROS doubles에서 기본 비활성·정상 활성·설정 파일 없음·worker 오류 모두 기존 accel/brake/steering/state가 유지되었다.
- 재생 도구는 기록된 입력을 다시 계산해 저장된 출력과 비교한다. 원래 출력이 invalid였던 기록도 같은 invalid 결과가 나오면 재생 일치에 포함되므로, 일치를 주행 성공으로 해석하지 않는다.

추가 점검에서 snapshot 오류가 첫 frame에만 실려 대기열 교체 시 누락될 수 있는 경우를 재현했다. 새 요청이 들어올 때까지 오류 표시를 유지하도록 수정하고 회귀 검사를 추가했다. 현재 실행의 보존 보고서는 [SHADOW_VALIDATION_SNAPSHOT_FIX_2026-10-25.json](SHADOW_VALIDATION_SNAPSHOT_FIX_2026-10-25.json)이다. 소스 SHA256과 검사 수를 포함하며, [직전 369개 검사 보고서](SHADOW_VALIDATION_2026-10-25.json)는 당시 기록으로 유지한다.

## 남은 범위

실제 센서 bag/로그 확보 및 재생, Linux catkin 빌드·native ROS 메시지·동일 tick 입력/시각 확인, 차량 파라미터·조향 지연·부하·추종 오차와 MORAI 전환 주행은 미검증이다. 새 조향을 실제 명령으로 채택하는 변경과 기준 자료 폴더 삭제는 수행하지 않았다. 1단계 전체 완료로 처리하지 않는다.
