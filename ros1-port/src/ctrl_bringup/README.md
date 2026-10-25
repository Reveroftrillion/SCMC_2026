# CTRL 팀 실행 구성 — ctrl_bringup

여러 패키지의 노드·설정·지도·경로를 하나의 ROS1 실행 구성으로 묶는다. 이 패키지가 미션 알고리즘 자체를 새로 구현하는 것은 아니다.

| 위치 | 용도 |
| --- | --- |
| [launch/ctrl_competition.launch](launch/ctrl_competition.launch) | 센서·위치·인지·계획·제어 연결 |
| [config/low_speed.yaml](config/low_speed.yaml) | 기존 실행 노드의 설정 |
| [routes/](routes/) | 전역/시작 연결 경로와 체크포인트 |
| [maps/](maps/) | 도로 지도와 관련 캐시 |

호스트별 포트는 [docker/local.launch](../../docker/local.launch)에서 덮어쓴다. 실행은 [ROS1 안내](../../README.md)를 따르며 새로운 패키지 이름으로 빌드한 환경을 사용한다. `.launch.py`는 보존된 ROS2 형태의 참고 파일이고 현재 ROS1 실행은 XML `.launch`를 사용한다.

지도·경로·체크포인트의 출처는 [자산 목록](../../docs/independent_assets_2026-10-25.json)에 기록한다. 사용 근거가 확인되지 않은 파일을 CTRL 팀이 직접 측량한 데이터로 표시하지 않는다.
