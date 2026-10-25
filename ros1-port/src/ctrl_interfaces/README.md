# CTRL 팀 메시지 계약 — ctrl_interfaces

센서·인지·계획·제어 사이의 값과 단위를 정의한다. 메시지 정의는 알고리즘 작성 주체와 구분되는 공개 연결 계약으로 관리한다.

| 메시지 | 의미 |
| --- | --- |
| [VehicleStatus](msg/VehicleStatus.msg) | 속도 m/s, yaw rad, 모드·기어와 조향 피드백 |
| [Event](msg/Event.msg) | 관측 물체·신호와 frame·시각·신뢰도 |
| [DrivingTarget](msg/DrivingTarget.msg) | 속도 상한·정지 station·회피·atomic reference |
| [ControlCommand](msg/ControlCommand.msg) | 가감속과 조향각 rad, 생성 시각 |

`stop_s`의 앞바퀴 기준, 경로의 후륜축 기준, local 거리와 global station의 차이는 [제어 명세](../../docs/CONTROL_CONTRACT_2026-10-25.md)를 따른다. 만료나 비정상 수치를 유효한 제어 명령으로 바꾸지 않는다.

패키지 이름이 `ctrl_interfaces`로 변경됐으므로 catkin 메시지를 재생성하고 소비 노드도 같은 빌드 환경을 사용한다. 이번 문서 정리에서는 `.msg` 파일이나 필드를 변경하지 않았다.
