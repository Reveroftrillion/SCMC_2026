# Humble/Noetic 메시지 생성 기본값 점검

원본 production Python의 import와 constructor를 AST로 확인한 직접 생성 타입은 17개다: `Point`, `PointStamped`, `PoseStamped`, `TransformStamped`, `Odometry`, `Path`, `CompressedImage`, `Imu`, `NavSatFix`, `PointCloud2`, `PointField`, `Bool`, `ColorRGBA`, `Float64`, `String`, `Marker`, `MarkerArray`. 중첩된 Header, Quaternion, Pose/PoseWithCovariance, Transform, Twist/TwistWithCovariance, Vector3, NavSatStatus도 함께 확인했다.

| 항목 | Humble → Noetic | 이식 대응 |
| --- | --- | --- |
| Quaternion.w 기본값 | 1 → 0 | `foscar_ros1.messages.Marker`가 명시 pose 없는 Marker에만 w=1 적용 |
| Imu/NavSatFix covariance 9개, Odometry covariance 36개 | 모두 0 → 모두 0 | 원본 그대로 |
| NavSatStatus.status/service | 0/0 → 0/0 | 원본 그대로; 원본 bridge가 fix 상태와 GPS service를 명시 설정 |
| PointField.count/datatype/offset | 0 → 0 | 원본은 count=1과 자료형/offset 명시 설정 |
| Marker.id/type/action | 0 → 0 | 원본 그대로 |
| Marker.scale/color/lifetime/frame_locked | 0/0/0/false → 동일 | 원본 그대로; 활성 marker는 필요한 값 명시 설정 |
| 배열/문자열/Bool/Float64 | 빈 값/false/0 → 동일 | 원본 그대로 |
| uint8[] Python 저장 표현 | array.array → bytes | PointCloud2 생성 경계에서 bytes로 변환; 바이트·필드 배치 유지 |
| Marker 상수 선언형 | int32 → uint8 | Python 상수의 값은 동일; 실제 id/type/action 필드는 모두 int32 |

Quaternion 기본값에 의존해 발행하는 생성 위치는 DELETEALL Marker 7곳이다. 원본 위치:

- `foscar_lidar/obstacle_node.py:1805`
- `foscar_planning/behavior_node.py:1402`
- `foscar_planning/bev_node.py:128`
- `foscar_planning/highway_markers.py:8`
- `foscar_planning/mission_markers.py:91`
- `foscar_planning/prediction_markers.py:74`
- `foscar_planning/roundabout_markers.py:22`

다른 Marker와 Imu/Odometry/PoseStamped/TransformStamped의 발행 경로는 orientation을 명시 설정한다. Marker 어댑터는 명시 pose를 바꾸지 않고 native ROS1 클래스의 상수·타입·MD5·직렬화를 상속한다. 실제 생성 메시지 직렬화 왕복은 타깃 환경 검사기에 포함한다.

표준 메시지의 버전별 wire schema 차이도 있다. Noetic Header에는 추가 `seq`가 있고, Humble Marker에는 Noetic에 없는 `texture_resource`, `texture`, `uv_coordinates`, `mesh_file`이 있다. 원본은 이 필드를 읽거나 쓰지 않는다. 원본이 사용하는 stamp/frame/pose/marker 필드 의미를 보존한다. Header sequence는 이식본 판단 입력으로 사용하지 않는다.

판단 근거는 공식 정의다: [Humble Quaternion](https://raw.githubusercontent.com/ros2/common_interfaces/humble/geometry_msgs/msg/Quaternion.msg), [Noetic Quaternion](https://raw.githubusercontent.com/ros/common_msgs/noetic-devel/geometry_msgs/msg/Quaternion.msg), [Humble Imu](https://raw.githubusercontent.com/ros2/common_interfaces/humble/sensor_msgs/msg/Imu.msg), [Noetic Imu](https://raw.githubusercontent.com/ros/common_msgs/noetic-devel/sensor_msgs/msg/Imu.msg), [Humble NavSatFix](https://raw.githubusercontent.com/ros2/common_interfaces/humble/sensor_msgs/msg/NavSatFix.msg), [Noetic NavSatFix](https://raw.githubusercontent.com/ros/common_msgs/noetic-devel/sensor_msgs/msg/NavSatFix.msg), [Humble PointField](https://raw.githubusercontent.com/ros2/common_interfaces/humble/sensor_msgs/msg/PointField.msg), [Noetic PointField](https://raw.githubusercontent.com/ros/common_msgs/noetic-devel/sensor_msgs/msg/PointField.msg), [Humble Marker](https://raw.githubusercontent.com/ros2/common_interfaces/humble/visualization_msgs/msg/Marker.msg), [Noetic Marker](https://raw.githubusercontent.com/ros/common_msgs/noetic-devel/visualization_msgs/msg/Marker.msg), [Humble ColorRGBA](https://docs.ros.org/en/humble/p/std_msgs/msg/ColorRGBA.html), [Noetic ColorRGBA](https://raw.githubusercontent.com/ros/std_msgs/noetic-devel/msg/ColorRGBA.msg), [Noetic genpy 직렬화 생성기](https://github.com/ros/genpy/blob/noetic-devel/src/genpy/generator.py).
