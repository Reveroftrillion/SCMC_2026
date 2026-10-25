"""`foscar_camera_train` 을 소스트리에서 바로 import 한다 (colcon build 없이 돌리려고)."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
