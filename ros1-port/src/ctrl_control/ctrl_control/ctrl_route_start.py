"""Explicit first-lap initialization; default route projection remains unchanged."""
import math
from pathlib import Path
import re

import numpy as np


def start_station_hint(route, x, y, enabled, checkpoints_file):
    if not enabled:
        return None
    text = Path(checkpoints_file).read_text()
    match = re.search(r'반경\s*(\d+(?:\.\d+)?)\s*m', text)
    if match is None:
        raise ValueError('START radius missing from checkpoint geometry')
    radius = float(match[1])
    if (x is None or y is None or not all(math.isfinite(v) for v in (x, y, radius))
            or radius <= 0 or not np.array_equal(route.points[0], route.points[-1])
            or math.hypot(x-route.points[0, 0], y-route.points[0, 1]) > radius):
        raise ValueError('start_at_beginning requires the closed global route and a pose inside START radius')
    return 0.
