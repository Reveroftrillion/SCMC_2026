import numpy as np

with open('/home/foscar/Simulator_2025/src/planning/paths/25molit_ai_mission.txt', 'r') as f:
    path = []
    for line in f.readlines():
        x, y, z = map(float, line.split())
        path.append((x, y))

path = np.array(path)

dx = np.array([0] + list(np.diff(path[:, 0])))
dy = np.array([0] + list(np.diff(path[:, 1])))

dist = np.hypot(dx, dy)
path = path[dist != 0, :]

with open('/home/foscar/Simulator_2025/src/planning/paths/25molit_ai_mission_interpolated.txt', 'w') as f:
    for (x, y) in path:
        f.write(f"{x} {y} 0\n")
