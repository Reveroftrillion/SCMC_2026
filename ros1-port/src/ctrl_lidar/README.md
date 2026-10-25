# CTRL 팀 LiDAR 인지 — ctrl_lidar

포인트클라우드에서 지면과 물체를 구분하고 관측을 planning 계층에 전달한다. 현재 실행은 외부 참고 구현을 ROS1으로 이식·수정한 경로이며 새 인지 재구현은 후속 5단계다.

## 처리와 책임

`ctrl_lidar_bridge`가 센서 입력을 발행하고 `ctrl_obstacle_node`가 지면 분리·군집·외곽/추적·도로/가시성 정보를 처리한다. 주요 보조 모듈은 `ctrl_ground`, `ctrl_observed_footprint`, `ctrl_road_mask`, `ctrl_clearance`, `ctrl_tunnel_walls`다.

인지 노드는 관측과 관측 신뢰도를 전달한다. 우회/양보·속도·정지 판단은 planning과 control의 책임이다. 관측되지 않았다는 사실을 물체가 없다는 검증과 혼동하지 않는다.

## 실행과 설정

통합 실행은 [ROS1 안내](../../README.md)를 사용한다. 대회 구성이 [ctrl_obstacle.launch](launch/ctrl_obstacle.launch)에 값을 전달한다. 현재 호스트의 수신 포트는 [local.launch](../../docker/local.launch)의 2368이며 패키지 단독 기본 포트와 구분한다.

센서 높이·장착 TF, ground/ROI/DBSCAN 설정과 mission 시각의 유효성을 함께 확인한다. 예전 문서의 타인 노트북 IP나 기본 숫자를 현 차량에 검증된 값으로 옮기지 않는다.

## 오탐 분석과 재구현

[오탐 분석 절차](오탐분석.md)는 raw scan에서 입력·지면·외곽·판단 단계를 따라가는 순서를 설명한다. [회귀 fixture 설명](test/data/README.md)은 원래 측정 출처와 좌표를 보존한 자료다.

기존 지면 보정 결과는 [정적 정지 기록](../../docs/STATIC_STOP_2026-10-23.md), [터널 기록](../../docs/TUNNEL_CONTINGENCY_2026-10-23.md)에 당시 조건과 함께 있다. 새 구현은 다른 배치/높이/가림·관측 공백에서도 별도 평가해야 한다.

## 이전 문서 원문

[교체 전 원문과 당시 기록](../../../%EA%B3%BC%EA%B1%B0%EC%9E%90%EB%A3%8C/ctrl-document-history-20261025/ros1-port/src/ctrl_lidar/README.md). 원래 작성자·날짜·출처와 실험 수치를 보존했다.
