# Braking ground regression fixture

`braking_ground_20260922.npz`: original XYZ returns in lidar frame from
`ped_raw_20260922_trackfix20`, header timestamp1790045854.4308333.
The measured mounting is (2,0,2.11)m. This frame contains the real pedestrian
near base x7.42/y0.65, and thin elevated ground returns near x15.81/y-1.43
and x13.17/y-0.99 which previously entered static avoidance memory.
The test exercises raw point processing, including preservation of the person.
