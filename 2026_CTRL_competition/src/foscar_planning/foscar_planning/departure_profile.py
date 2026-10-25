"""MORAI full-pedal departure hypotheses (entry_accel_max 1.0), from recorded launches.

2026-09-29 standstill entries at full pedal: 5.2-5.3 m/s2 over the first second, 3.3-3.8 over two (the ring's
curvature speed caps it), ~0.2 s from the admission to moving. The .60-throttle set (2.9/3.4/4.0, 0.05-0.1 s)
under-rated our launch by half and held clear gaps against a car still well behind."""
ACCELERATIONS=(4.,4.6,5.2)
DELAYS={4.:.2,4.6:.175,5.2:.15}


def launch_delay(accel):
    # Retain compatibility with historical low-throttle replay hypotheses.
    return DELAYS.get(accel,.25 if accel<=1.5 or accel==2.5 else .2 if accel<=1.8 or accel==3. else .1)
