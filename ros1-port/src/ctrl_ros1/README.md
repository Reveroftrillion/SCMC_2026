# CTRL 팀 ROS1 호환 계층 — ctrl_ros1

외부 ROS2 참고 노드가 사용하는 일부 API를 ROS1 `rospy`에 연결한다. 이 계층의 이식 작업과 주행 알고리즘의 새 구현을 구분한다. production 코드에 가짜 `rclpy` 패키지를 설치하는 방식이 아니라 `ctrl_ros1`을 명시적으로 import한다.

## 책임

| 영역 | 역할과 확인 사항 |
| --- | --- |
| 콜백 | subscription 대기열과 executor에서 application callback 실행 |
| 시간 | ROS1 시각/정수 ns, timer와 시계 pause/jump 연결 |
| 파라미터 | YAML/서버/CLI override와 타입, 점 구분 이름의 ROS1 인코딩 |
| 리소스 | catkin 패키지 위치 해석 |
| 메시지 | 필요한 기본값 차이를 경계에서 처리 |

새 제어의 순수 계산 모듈은 이 호환 계층을 import하지 않는다. 현재 실행 노드의 콜백·TF·네트워크 동작 검증과 오프라인 수학 검사를 같은 결과로 표시하지 않는다.

## 외부 구현과 라이선스

`ctrl_point_cloud2.py`는 [ROS2 공식 Humble 구현](https://github.com/ros2/common_interfaces/blob/2de367120bab435a5874fc69b96b348687509405/sensor_msgs_py/sensor_msgs_py/point_cloud2.py)에 기반하며 BSD 라이선스를 보존한다. 원래 비교 fixture는 [point_cloud2_humble.py](test/fixtures/point_cloud2_humble.py), 출처와 hash는 [provenance JSON](test/fixtures/point_cloud2_provenance.json)에 있다.

해당 외부 commit은 `2de367120bab435a5874fc69b96b348687509405`, fixture SHA256은 `79582d403a689d88468e064dcfc39a4d6748de3459bc0492084a4abb0831e7eb`다. 이것을 CTRL 팀의 새 점군 알고리즘으로 표시하지 않는다.

콜백 순회와 timer 대응의 참고는 [Humble executor](https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/executors.py), [rcl timer](https://github.com/ros2/rcl/blob/humble/rcl/src/rcl/timer.c)다. 실제 ROS1 native 직렬화/TF와 시각 변경의 동등성은 별도 Linux 검사로 확인해야 한다.

## 확인

저장소 루트에서:

```bash
python3 -m unittest discover -s ros1-port/src/ctrl_ros1/test -v
```

이 명령은 가능한 단위 범위를 확인하며 catkin 설치·실제 네트워크 부하·차량 주행을 증명하지 않는다. 기존 자세한 dispatcher/시간/메시지 차이 설명은 아래 보관 문서와 [ROS1 통합 기록](../../docs/ROS1_INTEGRATION.md)을 참고한다.

## 이전 문서 원문

[교체 전 원문과 당시 기록](../../../%EA%B3%BC%EA%B1%B0%EC%9E%90%EB%A3%8C/ctrl-document-history-20261025/ros1-port/src/ctrl_ros1/README.md). 원래 작성자·날짜·출처와 실험 수치를 보존했다.
