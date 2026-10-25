# BEV clearance diagnosis — 2026-10-06

Run: `20261006_174505`. No production changes or new simulator runs.

Truth OBB Euclidean body gaps (same origin conventions as loop_runner): s296.184=19.370m; s300.075=15.461m; s309.693=5.956m; s311.354=4.380m. Truck ID2 remains at 0km/h in the recorded run.

Fixed envelope: ego width1.892m +0.5m each side; grid0.25m (orientation-dependent raster conservatism up to a cell diagonal0.354m), guard one-cell station setback0.25m. These are distinct from future-motion inflation.

Motion reconstruction at recorded target ticks: unready region uses INIT_SPEED_SIGMA20m/s, GATE_SIGMA3, so even0.5s gives ±30m per map axis before scan age. Replayed samples s296.184 ±52.985m, s300.075 ±51.327m, s311.354 ±40.183m. This reproduces zero caps and recorded stop_s at those samples. Replay is diagnostic, not bit-exact: latest odometry used as sensor origin and60km/h pre-guard input cap instead of original hidden pre-guard target. Original logs independently show repeated predicted occupancy stops.

Not all red markers mean this same guard decision. Recorded avoid_choice texts fluctuate LEFT/RIGHT HITS BOX during fragmented observations; s309.69 LEFT CHOSEN+6, RIGHT BLOCKED HITS BOX/OFF ROAD. The stop marker is red whenever stop_s is finite, including signal stops. Actual final state is AVOID with v_max0, so final stop is not a red traffic-signal wait.

Do not fix this by globally reducing the0.5m margin: moving-region uncertainty is grossly larger. Stabilize motion evidence across split/merge and avoid treating the broad uninitialized KF prior as measured imminent collision. Separately, reconstructed selected curve intersects the full truth OBB arounds316.4 despite being clear against partial observed occupancy. This is a model-box proxy, not a collision replay; it means the whole maneuver cannot yet be declared safe just from current4.38m clearance. Hidden actual outline and tracking continuity need validation.

Artifacts: inspect_bev_clearance.py, bev_clearance.json, bev_clearance.svg.
