#!/usr/bin/env python3
"""Check actual bag persistence with drive=false; never enable control UDP."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rosbag

PORT = Path(__file__).resolve().parents[1]
out = (PORT/'docker/artifacts/drive-first-20261013/bag-stationary-check').resolve()
out.mkdir(parents=True, exist_ok=True)
environment = dict(os.environ, ROS_LOG_DIR=str(out/'ros-logs'), HOME=str(out/'home'))
Path(environment['HOME']).mkdir(exist_ok=True)
with (out/'launch.log').open('w') as log:
    child = subprocess.Popen(['roslaunch', str(PORT/'docker/local.launch'), 'drive:=false', 'bag:=true',
                              'bag_dir:=' + str(out/'bags')], env=environment, stdout=log,
                             stderr=subprocess.STDOUT, start_new_session=True)
    try:
        time.sleep(8)
    finally:
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            child.wait(timeout=20)
bags = []
for path in (out/'bags').glob('*.bag'):
    with rosbag.Bag(str(path)) as bag:
        bags.append(dict(path=str(path), bytes=path.stat().st_size, messages=bag.get_message_count(),
                         topics=sorted(bag.get_type_and_topic_info().topics)))
report = dict(scope='stationary logging check only; drive=false', launch_returncode=child.returncode,
              bags=bags, passed=bool(bags and all(x['messages']>0 for x in bags)))
(out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
raise SystemExit(0 if report['passed'] else 1)
