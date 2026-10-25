# 식별자·개인 개발 문서 정리 기록

작업 환경: Windows / Python3.10.11. 기준 브랜치 `feat/ros1-port`에서 깨끗한 상태를 확인하고
`refactor/identifiers-docs`를 생성했다. 작업 전 HEAD는 `c185c84024824fb5c2f4136e91a6b7b4e7bd02e7`이다.
코드 동작 변경·버그 수정·주행·commit·push·main 병합은 수행하지 않았다.

## 개발 목표와 영향 파일

런타임 Python **10개**, 함수/메서드/내포식 범위의 지역 식별자 **93개**와 관련 참조를 정리했다.
새 검증 도구 Python2개를 추가해 전체 Python 변경/추가 파일은12개다.

| 변경 파일 (`src/` 기준) | 정리 범위·예시 |
| --- | --- |
| `foscar_camera/foscar_camera/yolo_onnx.py` | 클래스 ID·유효 행·검출 index·box·영상 크기·resize 크기·ONNX 입력. `classes→class_ids`, `d→detection`, `nw→resized_width` |
| `foscar_camera/foscar_camera/stop_lines.py` | CSV stream·색·정지선 번호·route station·global/spawn route·offset. `out→stop_points`, `i→line_number` |
| `foscar_control/foscar_control/reference.py` | 전역 station·좌표 배열·투영 위치. `gs→global_stations`, `pts→reference_points` |
| `foscar_control/foscar_control/reference_control.py` | 이전 reference·전역 투영·프로파일 index·reference controller·인계 suffix·감속·출력. `old→previous_reference`, `inner→reference_controller` |
| `foscar_control/foscar_control/speed_policy.py` | 제한 구간의 시작/끝 station. `begin→start_s`, `end→end_s` |
| `foscar_lidar/foscar_lidar/ground.py` | RANSAC 난수·표본 index·최대 지지 inlier 집합과 수. `best→best_inliers` |
| `foscar_lidar/foscar_lidar/free_space.py` | ray-box 교차 parameter·잔여 거리·회전·차체 좌표·셀 경계. `o→local_origin`, `lo→cell_lower` |
| `foscar_planning/foscar_planning/signal_stop.py` | 정지선·교차로 ID·앞축 정지 목표. `line→stop_line`, `stop→front_stop_s` |
| `foscar_ros1/foscar_ros1/executors.py` | ready snapshot·wall 대기 시간·timer 잔여 ns·미완료 Future. `delay→wait_seconds` |
| `foscar_ros1/foscar_ros1/_entities.py` | timer deadline 초과량. `ahead→overdue_ns` |

정확한 함수별 이름 대응은 [check_identifier_refactor.py](../tools/check_identifier_refactor.py)의 `RENAMES`에 있다.
AST의 Name 위치만 치환해 문자열·속성·keyword·함수/클래스/인자명을 보존했고 한글 주석은 별도로 수정했다.
새 이름 충돌, 지역 바인딩 여부, 공개 인자 제외, 이전 참조 잔존 여부와 역매핑 AST 전체 일치를 검사했다.
계산식·연산 순서·분기·루프·상수·튜닝·속도 제한·토픽·메시지 필드·YAML/launch·경로·포트·직렬화는 변경하지 않았다.
앞축/후륜축 s, ROI 모순 색, ray parameter와 거리, ground inlier 선택, timer/snapshot 주석을 실제 구현에 맞췄다.

동적 참조의 `_TRANSFER` 문자열 및 `getattr/setattr` 대상 속성은 유지했다. 외부 접근 가능한 상태 속성,
메시지/dataclass 필드, callback·entry point, 공개 인자와 라이선스·출처·저작권은 변경하지 않았다.
짧은 수학 좌표·단위 관례 식별자는 의미가 명확하면 보존했다. 나머지 Python 전체에 기계적 일괄 치환을 적용하지 않았다.

## Markdown과 개인 개발 지침

- 수정: [루트 README](../../README.md), [ROS1 README](../README.md).
- 신규: [AGENTS.md](../../AGENTS.md), [DEVELOPMENT_WORKFLOW.md](DEVELOPMENT_WORKFLOW.md), [MARKDOWN_INDEX.md](MARKDOWN_INDEX.md), 이 기록.
- 현재 계획·과거 인계·팀 계획·출처 문서를 구분하고 Windows와 Ubuntu의 수행 범위를 명시했다.
- workflow에 전체 구조, 구현/검증, 재현 문제, 우선순위, 환경별 명령, 완료 기준, Git/변경 기록/인계를 포함했다.
- 교차로2는 **미해결**이다. Camera-7→YOLO box→ROI→신호 선택→Behavior→DrivingTarget 순서의 증거 확보를 P0로 두었다.
- 과거 팀 문서의 역할·작성자·출처·체크박스, 지정된 날짜별 검증 문서, 기존 JSON·SHA256 manifest를 수정하지 않았다.

## 검증 방법과 결과

재현 명령은 저장소 루트 PowerShell에서:

```powershell
ros1-port/.python310/python.exe ros1-port/tools/check_identifier_refactor.py
ros1-port/.python310/python.exe ros1-port/tools/run_refactor_checks.py
```

최종 검사 artifact는 `verification/.work/identifier-refactor-20261013-205930/`에 있다.
경로 시각은 실행 호스트가 생성한 이름이며 현장 bag의 KST/UTC 시각과 혼용하지 않는다. `.work/`는 Git 제외다.

| 검사 | 결과 |
| --- | --- |
| Python 구문 | 추적 Python250개 + 신규 도구2개, 총252개 compile PASS. pyc 생성 없이 확인 |
| 역매핑 AST | 런타임10개·식별자93개 PASS. 고정 기준 커밋과 AST 전체 일치 |
| ROS1 adapter | 27개 중26 PASS, native 메시지 직렬화1 skip. fake rospy 테스트이며 실제 ROS 실행 아님 |
| packaging 보조 단위 | 8 PASS |
| packaging 정적 | launch 인자110·노드21·typed parameter106·bag24+2·resource/message21 PASS |
| 원본/이식 차등 | 각각909 PASS, trace35,608개·수치3,657,429개, 최대 차이0, 동일 trace SHA256 `1c88a25cdec77b28a5a99ac8ce585660009a34b57d8b1fa91f57e15699e4c36a` |
| 실제 ONNX 합성 비교 | 3개 이미지 raw/decode/선택색 차이0 PASS. CPUExecutionProvider, 검출 결과 비어 있음 |
| 원본/과거 기록 해시 | runner가 원본530개를 포함한 보호 파일604개 전후 SHA256 비교, 변경0 |
| 기존 preservation | FAIL(exit1). 원본530개 manifest 일치, 기존 테스트·fixture99개 동일. production8개에서 의도된 식별자 차이 |
| 기존 port_boundary | FAIL(exit1). 위 production8개의 이름/주석 차이. 변환 기준을 수정하거나 --write로 덮어쓰지 않음 |
| 기존 preflight | FAIL(exit1). 기존 모델 README 해시 불일치와 이번 boundary 불일치를 함께 보고 |
| Markdown 정적 | 154개 분류, 현재 문서6개의 내부 링크240개·명령/리소스 경로12개 PASS. 보존 문서 링크232개 중 기존 불일치19개는 원문 보존 |
| Git whitespace | 기본 `git diff --check` FAIL: 기존 CRLF를 trailing whitespace로 보고. 추가 `git -c core.whitespace=cr-at-eol diff --check` PASS. 줄바꿈/저장소 설정은 변경하지 않음 |

기존 preservation/boundary가 보고한 파일8개는 위 표의 camera2·control3·lidar2·planning1이다.
새 ROS1 adapter2개는 원본 Python byte 검사 대상에 포함되지 않으므로 AST와 adapter suite로 별도 확인했다.
이 실패들은 검사 기준 변경으로 성공 처리하지 않았다. 새 runner 반환값0은 필수 회귀 검사 통과를 의미하며 모든 기존 검사 PASS라는 뜻이 아니다.

최초 새 runner 실행 `identifier-refactor-20261013-204747/`에서는 테스트 대역의 adapter 상대 경로가 없어서
원본/이식 pytest가 수집 전 실패했다. 새 결과 저장 도구에서 상대 경로용 adapter 사본을 준비한 뒤 새 디렉터리에서 전체 재실행했다.
기존 테스트·기대값·검사 기준은 바꾸지 않았고 최초 실패 로그도 보존했다.
그 뒤 주석의 한글 인코딩을 바로잡고 역매핑 AST를 다시 확인했다. 실행 AST 변경이 없어 전체 suite의 추가 반복은 하지 않았다.

최종 검토에서는 reference의 `global_s+s_offset` 결과를 더 정확히 설명하도록 지역변수 `route_stations`로 정리했다.
이 추가 이름 변경 후에는 위 최종 artifact 디렉터리에서 전체 회귀 검사를 다시 실행해 같은 성공 수치와 차이0을 확인했다.
보호 파일604개의 전후 해시도 다시 일치했다. 각 차등 suite에는 기존 경고10개가 있었고 실패는 없었다.

## 호환성 한계와 다음 Ubuntu 확인

이번 세션에서 ROS1 master·catkin 빌드·실제 메시지 직렬화·TF·GPU·MORAI 주행은 실행하지 않았다.
Windows 공통 대역의 차등 비교는 실제 도착 순서·스케줄·TCPROS/DDS·clock·네트워크·센서 장착 동등성을 입증하지 않는다.
합성 영상 검출 없음은 실제 교차로 녹색 선택의 증거가 아니다.

다음 Ubuntu에서는 새 코드의 devel/install·실제 adapter suite를 다시 확인하고, Competition9088→909 및 drive=false 제어 UDP0,
Camera-7 원본/box/ROI/chosen/신호 stamp/Behavior/DrivingTarget의 같은 시각 기록을 확보한다.
원본 Camera-7 해상도·pitch와 위치/FOV·차량 좌표 근거를 대조한다. 신호 버그 수정·실제 출발/정지 검증은 별도 요청 범위다.
전체 미션 replay·전송 지연/손실·clock·종료 traceback·bag 파일명/로그 보존 검증도 남아 있다.
