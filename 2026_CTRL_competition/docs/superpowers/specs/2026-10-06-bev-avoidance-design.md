# BEV occupancy avoidance

User-approved direction: finish the first obstacle mission before the tunnel, replace enlarged object AABBs with small occupied BEV cells, and use occupancy motion rather than semantic static/dynamic labels. Reuse the roundabout FREE/OCCUPIED/UNKNOWN and edge Kalman approach. Prioritize short regression checks and direct MORAI over long offline sweeps. No commits without request.

The first integration consumes complete, time-coherent lidar footprint scans. Observed convex footprints are rasterized into intersected cells, never their enclosing rectangle. Separate pieces remain separate. The field retains scan visibility and marks hull shadows/range as unknown, following RingLane. Motion uses spatially associated occupied regions and EdgeTracks in both planar axes, with hidden edges excluded and uncertainty retained. No object kind or object ID decides whether to avoid or stop.

A candidate is accepted only when the vehicle swept body stays in mapped pavement and avoids occupied/unknown space up to the currently validated horizon. Temporal predictions are evaluated over the ego arrival-time interval, with speed uncertainty; slowing invalidates and rechecks the timing. Preserve observed geometry through occlusion without leaving a stationary trail behind a moving region. No BEV-driven tunnel activation in this iteration.

Important limitation: footprint input preserves observed shape but is not a raw-point visibility grid; never claim unobserved concavities inside a footprint are free. Explicitly surface that limitation in results. First prove scan delivery, cell geometry, motion estimation and a bounded planning check before MORAI.
