import numpy as np
import sys
import os

# Add current directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import frenet_optimal_trajectory as fot

# gps_offset_x, gps_offset_y = 313008.55819800857, 4161698.628368007 
g_path = []
with open('/home/foscar/Desktop/GITAE/Simulator_2025/src/planning/paths/25molit_main_mission.txt', 'r') as f:
    while True:
        line = f.readline()
        if not line: break

        parts = line.split()
        if len(parts) < 2:
            continue
        x, y = float(parts[0]), float(parts[1])
        g_path.append((x, y))

g_path = np.array(g_path)

# Smooth the raw path using moving average filter
def smooth_path(path, window_size=5):
    """Apply moving average smoothing to reduce jaggedness"""
    smoothed = np.copy(path)
    half_window = window_size // 2

    for i in range(half_window, len(path) - half_window):
        smoothed[i] = np.mean(path[i-half_window:i+half_window+1], axis=0)

    return smoothed

# Apply smoothing (increase window_size for smoother path: 5, 7, 9, 11...)
g_path = smooth_path(g_path, window_size=7)

import matplotlib.pyplot as plt

_, _, _, _, csp = fot.generate_target_course(g_path[:, 0], g_path[:, 1])

s = csp.s[-1]
# Smoother path with finer resolution (0.1m instead of 0.2m)
s_values = np.arange(0, s, 0.2)
path = np.array([csp.calc_position(s) for s in s_values])
yaw = np.array([csp.calc_yaw(s) for s in s_values])

d = 10

new_path = np.copy(path)
new_path[:, 0] += d * np.cos(yaw + np.pi / 2)
new_path[:, 1] += d * np.sin(yaw + np.pi / 2)

# plt.plot(new_path[:, 0], new_path[:, 1], marker='o')
# plt.plot(path[:, 0], path[:, 1], marker='o')
# plt.show()

with open('/home/foscar/Desktop/GITAE/Simulator_2025/src/planning/paths/f.txt', 'w') as f:
    for i in range(len(path)):
        f.write(f'{path[i][0]} {path[i][1]} {yaw[i]} {9}\n')
