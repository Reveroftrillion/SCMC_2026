# Camera regression images

`front_road_without_stop_line.png` is a previously received 1280×720 image from the
original road-facing front camera (UDP 9291). The front ROI must reject it as having
no stop line. Positive front-camera stop-line tests currently use synthetic images;
real stop-line scenes from this camera still need driving validation.

## Historical Camera-7 scenes

Camera-7 is now dedicated to traffic lights at pitch -20°. The following images
preserve regressions from its former stop-line role and use the old `stop_line.yaml`.

MORAI RGB frames received from the user's stop-line camera at 640×480,
mount [1.9, 0.0, 1.2], pitch +12.5°, horizontal FOV 90°.

- `camera7_stop_line_near.jpg`: stop line at image y ≈ 0.583, immediately above the bonnet.
  The original broad ROI clipped the line and produced no candidate.
- `camera7_stop_line_approach.jpg`: stop line at image y ≈ 0.457, before the direction arrow.
  A clipped part of the arrow at the lower ROI boundary must not replace the stop line.
- `camera7_road_without_stop_line.jpg`: no stop line in the road ROI; a distant bright curb
  above the ROI must not become a stop-line candidate.
- `camera7_arrow_before_stop_line.jpg`: a full three-way direction arrow at y ≈ 0.498
  previously replaced the actual stop line at y ≈ 0.432. The central-stem filter
  must exclude the arrow branch and preserve the stop line ahead.

These scenes verify specific regressions, not general perception performance or metric distance.
