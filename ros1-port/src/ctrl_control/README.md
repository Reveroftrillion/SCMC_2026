# CTRL 팀 제어·위치 추정 — ctrl_control

차량 상태를 받아 추종 명령을 계산하고 센서·제어 통신을 연결한다. 현재 실행 코드와 새 오프라인 계산을 구분한다.

| 구분 | 주요 모듈 | 상태 |
| --- | --- | --- |
| 기존 실행 | `ctrl_bridge`, `ctrl_localization`, `ctrl_follower`, `ctrl_core` | 외부 참고 구현의 ROS1 이식·수정 경로 |
| 새 조향 | `ctrl_lateral` | 독립된 계산 API, 오프라인 검증 |
| 새 진행 관리 | `ctrl_progress` | 구간 재획득·오차·시간 관리 |
| 새 경로 변환 | `ctrl_path_conversion` | 일반/회피/reference와 전역 거리 대응 |
| 새 요청 선택 | `ctrl_path_selection` | 선택·유지·해제·만료와 오류 차단 |
| 새 비교 기록 | `ctrl_shadow` | follower 입력 병행 계산·JSONL 기록·재생, 기본 비활성 |

새 계산은 Python 표준 라이브러리와 새 모듈끼리의 의존성을 사용한다. follower의 `shadow_config_file`에 설정 파일을 지정하면 병행 계산을 기록한다. 기본값은 빈 문자열이며 실제 명령은 기존 제어기가 계산한다. 실제 차량의 조향 응답은 아직 검증하지 않았다.

## 인터페이스

위치는 후륜축 기준 m, yaw와 조향각은 rad, 내부 속도는 m/s다. `DrivingTarget.stop_s`는 앞바퀴 정지 전역 station이므로 기준점 변환을 명시해야 한다. 새 polyline 거리는 전역 station과 다를 수 있다.

[제어 계약](../../docs/CONTROL_CONTRACT_2026-10-25.md)과 [메시지 안내](../ctrl_interfaces/README.md)를 기준으로 연결한다. 실물 파라미터·정규화 명령 변환은 기존 보정값을 무조건 정답으로 쓰지 않고 별도 확인한다.

## 검증

저장소 루트에서:

```powershell
& .\ros1-port\.python310\python.exe -X utf8 -B .\ros1-port\tools\validate_shadow.py
```

[현재 결과](../../docs/SHADOW_RESULT_2026-10-25.md)에 370개 검사와 합성 기록 재생, 미검증 범위가 있다. 실행/재빌드는 [ROS1 안내](../../README.md)를 따른다. 새 모듈을 계산한다고 기존 주행 전체가 CTRL 독자 구현으로 교체된 것은 아니다.
