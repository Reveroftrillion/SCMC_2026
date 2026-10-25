# foscar_camera — 카메라 7번 YOLO 신호 인식

카메라는 **7번 하나만** 쓴다. 영상에서 신호등 색을 YOLO(ONNX)로 읽어 `/perception/traffic_signal` 로 내보내고,
정지는 `behavior_node` 가 한다 (`/behavior/target` 발행자는 behavior_node 하나).
동작·정지 위치·시험 순서의 상세는 [YOLO_STOPLINE_2026-09-28.md](YOLO_STOPLINE_2026-09-28.md).

## 실행

```bash
# 최초 1회: onnxruntime(-gpu) 등은 워크스페이스의 .camera-deps 에 설치한다 (GPU 가 있으면 YOLO 가 CUDA 로 돈다)
python3 -m pip install --target .camera-deps -r src/foscar_camera/requirements-onnx.txt
colcon build --symlink-install

bash run_competition.sh                    # 대회: 카메라 기본 켜짐 (camera:=true)
bash run_camera.sh                         # 카메라 확인용: 제어 송신 없음 (drive:=false)
ros2 launch foscar_camera yolo_signal.launch.py    # 카메라 브리지 + YOLO 만 단독
ros2 run foscar_camera camera_view                 # 영상 창 (디버그 오버레이, s 키 = 원본 저장)
```

`run_competition.sh`·`run_camera.sh` 는 `.camera-deps` 를 PYTHONPATH 앞에 둔다. 이게 없으면 YOLO 노드가 onnxruntime 을 못 찾는다.

## 노드

| 실행 파일 | 역할 | 입력 → 출력 |
|---|---|---|
| `camera_bridge` | MORAI 카메라 UDP 조각을 이어붙여 JPEG 그대로 발행 | UDP :9208 → `/sensors/camera/traffic/compressed` (frame `camera_7`) |
| `yolo_signal_node` | 정지선 촬영 구간에서만 YOLO 추론, 2프레임 연속 같은 색이면 채택 | 위 토픽 + `/localization/s` → `/perception/traffic_signal`, 디버그 영상, `~/foscar_logs/yolo/*_cam7.jsonl` |
| `camera_view` | 영상 창. 디버그 영상이 없으면 원본 | `s` 키: 원본 프레임 `~/foscar_logs/cam_*.jpg` 저장 |
| `gps_logger` | 키를 누른 순간의 GPS 위경도·map 좌표를 CSV 에 기록 (정지선 측량용, `gps_record.launch.py`) | `/sensors/gps` → `~/foscar_logs/gps_<시각>.csv` |

설정: 정지 위치·ROI 는 [config/yolo_stoplines.json](config/yolo_stoplines.json), 인식 파라미터는 [launch/yolo_signal.launch.py](launch/yolo_signal.launch.py), 모델은 `models/best.onnx`.
`config/stop_lines.csv` 와 `stop_lines.py` 는 데이터셋 도구(`foscar_camera_train`)가 쓴다.

## 시뮬 설정

MORAI 센서 Network Setting 에서 **카메라 7번 → 노트북 192.168.0.10:9208** 이 Connect 되어 있어야 한다 (1280×720, pitch 335°).
`camera_bridge` 로그가 `아직 영상 없음` 이면 포트를, `깨진 장` 이 계속 나오면 `aux_header_bytes` 를 의심한다.

## MORAI 카메라 UDP 포맷 (실측)

```
'MOR' 3 byte | sec 4 | nsec 4 | index 4 | size 4 | JPEG 조각 (≤ 64979) | 꼬리 2 byte
```

- 한 장이 여러 조각으로 오고 **마지막 조각만 꼬리가 `EI`**. 이어붙인 결과가 `FF D8 … FF D9` 인지 확인해 깨진 장은 버린다.
- 옛 MORAI 예제처럼 부가 헤더가 index 4 byte 뿐인 버전이면 `aux_header_bytes: 4`.

## 확인한 것

- 2026-09-29 풀랩 (camera:=true): PASS 189.6 s. YOLO 처리 106 프레임 (red 67 / green 13 / yellow 7).
- 모델 메타데이터 설명 문자열만 바꾼 뒤에도 원본과 추론 결과가 같다 (이미지 4장, 검출·신뢰도 동일).
