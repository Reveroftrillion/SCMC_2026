#!/usr/bin/env python3
"""Prepare first-lap audit launch copies with persistent logs; never start nodes.

Run inside the workspace container, then pass the printed launch to verify_live
and audit_course. Only includes, log destinations and explicit first-lap options
are changed. Production launch files and HOME are preserved.
"""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


def prepare(output):
    output = output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    sources = {
        'camera': ROOT/'src/ctrl_camera/launch/ctrl_yolo_signal.launch',
        'delayed': ROOT/'src/ctrl_bringup/launch/ctrl_competition_delayed.launch',
        'competition': ROOT/'src/ctrl_bringup/launch/ctrl_competition.launch',
        'local': ROOT/'docker/local.launch',
    }
    replacements = {
        'delayed': ('$(find ctrl_camera)/launch/ctrl_yolo_signal.launch', str(output/'camera.launch')),
        'competition': ('ctrl_bringup ctrl_competition_delayed.launch', str(output/'delayed.launch')),
        'local': ('$(find ctrl_bringup)/launch/ctrl_competition.launch', str(output/'ctrl_competition.launch')),
    }
    for name, source in sources.items():
        content = source.read_text()
        if name in replacements:
            old, new = replacements[name]
            if content.count(old) != 1:
                raise ValueError('Unexpected launch structure: '+str(source))
            content = content.replace(old, new)
        root = ET.fromstring(content)
        if name == 'camera':
            param = root.find("./node[@name='yolo_signal_node']/param[@name='detections_log']")
            if param is None:
                raise ValueError('YOLO log parameter missing')
            param.attrib = dict(name='detections_log', value='$(env ROS_LOG_DIR)/yolo.jsonl', type='str')
        elif name == 'competition':
            follower = root.find("./node[@name='path_follower']")
            if follower is None or follower.find("./param[@name='log_dir']") is not None:
                raise ValueError('Unexpected follower log configuration')
            ET.SubElement(follower, 'param', name='log_dir',
                          value='$(env ROS_LOG_DIR)/follower', type='str')
        (output/(name+'.launch')).write_text(ET.tostring(root, encoding='unicode')+'\n')
    root = ET.Element('launch')
    for name in ('drive', 'bag'):
        ET.SubElement(root, 'arg', name=name, default='false')
    include = ET.SubElement(root, 'include', file=str(output/'local.launch'))
    for name in ('drive', 'bag'):
        ET.SubElement(include, 'arg', name=name, value='$(arg '+name+')')
    for node in ('localization', 'path_follower', 'behavior_node'):
        ET.SubElement(root, 'param', name='/'+node+'/start_at_beginning', value='true', type='bool')
    launch = output/'from-start.launch'
    launch.write_text(ET.tostring(root, encoding='unicode')+'\n')
    manifest = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in sources.values()}
    (output/'source-hashes.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return launch


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    print(prepare(parser.parse_args().output))
