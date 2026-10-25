# 변경 파일 안내

2026-10-22 이후에는 사용자 승인으로 지면/터널 회피, 첫 랩 초기화,
신호 ROI, 오른쪽 정적 경로 및 보행자 지면 보정을 최소 수정했다.
따라서 아래 최초 이식의 보존/동등성 설명을 현재 소스 전체의 바이트 동일
주장으로 해석하지 않는다. 최신 변경·완주와 남은 문제는
[현재 상태](CURRENT_STATUS_2026-10-22.md), 검사 결과는
[검증 요약](verification-20261022/checks-summary.json)을 참고한다.

이식 구현과 검증 파일은 `ros1-port/` 아래에 있다. 저장소 루트의 [`README.md`](../../README.md)는 작업 요약·실행 방법·검증 결과·GitHub 업로드 명령을 안내한다. 원본과 다른 RMT 구현 폴더는 수정하지 않았다.

| 경로 | 역할 |
| --- | --- |
| `src/{foscar_bringup,foscar_camera,foscar_camera_train,foscar_control,foscar_interfaces,foscar_lidar,foscar_planning}/` | 원본 src 복사. 계산 코드·테스트·모델·지도·경로·설정을 보존 |
| 각 패키지 `CMakeLists.txt`, `package.xml`, `setup.py`, `scripts/` | catkin 빌드·메시지 생성·Python 실행 래퍼·리소스 설치 |
| `src/CMakeLists.txt` | 독립 catkin workspace 진입점 |
| `src/foscar_ros1/foscar_ros1/` | rospy 실행·큐·callback group·시계·파라미터·TF·PointCloud2·Marker 기본값 어댑터 |
| `src/foscar_ros1/test/` | 실제 어댑터 검사, 고정 upstream serializer fixture와 무결성 검사 |
| `src/foscar_bringup/launch/*.launch` | competition/low_speed 및 지연 camera include |
| `src/foscar_lidar/launch/*.launch`, `src/foscar_camera/launch/*.launch` | 원본 포함/선택 launch의 ROS1 XML |
| `src/foscar_bringup/scripts/{delay_wall,launch_info,record_bag}` | wall delay·기록 이름·원본 bag 토픽 보존 |
| `src/*/rviz/*.rviz` 3개 | ROS1 RViz 플러그인·topic/QoS 표현 변환 |
| `run_competition.sh` | Noetic/catkin 환경 실행, 원본 drive 기본값 보존 |
| `.gitattributes` | raw calibration/resource bytes의 Git 줄바꿈 변환 방지 |
| `.gitignore` | 로컬 Python 및 임시 검증 복사본 제외 |
| `tools/port_boundary.py` | 허용된 production Python 변경의 재현·검사 |
| `tools/{check_packaging,test_packaging,preflight,check_noetic_environment,check_model_equivalence}.py` | 실행 구성·리소스·대상 환경·실제 모델 비교 검사 |
| `tools/generate_packaging.py`, `tools/resource_hashes.json` | 원본 launch 계약에 따른 패키징 생성 및 리소스 해시 기준 |
| `tools/bootstrap_windows310.py`, `tools/requirements-windows310.txt` | 시스템 변경 없이 같은 Windows 검증 환경 재구성 |
| `verification/*.py`, `verification/*.json` | 기존 테스트 재사용, 동일 입력/시각·결과 비교, 원본 전체 해시와 결과 |
| `README.md`, `docs/` | 실행·보존 근거·원본 문제·검증 결과와 남은 통합 검증 |

26개 production Python 경계 변경 파일의 정확한 목록은 `verification/preservation-results.json`의 `adapted_files`에 있다. 그 외 production Python 78개와 기존 테스트/fixture 99개는 현재 원본과 바이트가 같다. 복사한 ROS2 `*.launch.py`는 변경하지 않은 비교 fixture이며 실행 대상은 ROS1 `*.launch`다.
