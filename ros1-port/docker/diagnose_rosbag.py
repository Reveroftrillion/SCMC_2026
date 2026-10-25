#!/usr/bin/env python3
"""Read-only rosbag CLI diagnosis; no master, recording or control is started."""
import importlib
import shutil
import subprocess
import traceback

print('rosbag executable:', shutil.which('rosbag'), flush=True)
try:
    module = importlib.import_module('rosbag')
    print('rosbag import:', module.__file__, flush=True)
except Exception:
    traceback.print_exc()
subprocess.run(['rosbag', 'record', '--help'], check=False)
