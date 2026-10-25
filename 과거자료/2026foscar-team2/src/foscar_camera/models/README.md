# 사용자 제공 학습 모델

`best.onnx`: 2026-09-28 사용자 제공 파일 그대로 복사. SHA256 `04ab171d8a46b5576891a23034000fa34cc63bf63336c8edb67b9a3fee551b8b`.

메타데이터: Ultralytics 8.4.158 YOLO26s Detect, 2026-09-28T05:47:29.775838+00:00 export.
입력 images float32 [1,3,1280,1280], 출력 output0 [1,11,33600], end2end=False / NMS 미포함.
클래스: 0 red, 1 yellow, 2 green, 3 green_left, 4 left, 5 pedestrian, 6 red_left.
전처리: RGB, 114 letterbox, 0~1, NCHW. 후처리: xywh→xyxy, class score, NMS, 원본 좌표 복원.
모델에 기록된 라이선스: AGPL-3.0 (https://ultralytics.com/license).
