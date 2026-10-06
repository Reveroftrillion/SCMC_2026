"""Shared offline tool I/O. Importing this module never starts ROS nodes."""
import csv
import json
import math
from pathlib import Path
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/planning/scripts'))
DEFAULT_CONFIG = ROOT / 'src/planning/config/local_planner.yaml'
DEFAULT_PATH = ROOT / 'src/planning/paths/global_path.txt'
# Existing GlobalPathPlanner.py translation, not a new sensor estimate.
GLOBAL_MAP_OFFSET = (302595.0, 4124145.0)


class UniqueLoader(yaml.SafeLoader):
    pass


def unique_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise ValueError('duplicate YAML key: %s (line %d)' % (key, key_node.start_mark.line + 1))
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping)


def load_document(path):
    with Path(path).open(encoding='utf-8-sig') as stream:
        result = yaml.load(stream, Loader=UniqueLoader)
    if not isinstance(result, dict):
        raise ValueError('YAML root must be a mapping')
    return result


def planner_config(path):
    result = load_document(path).get('static_obstacle_avoidance_planner')
    if not isinstance(result, dict):
        raise ValueError('static_obstacle_avoidance_planner must be a mapping')
    return result


def finite_number(value):
    return type(value) in (int, float) and math.isfinite(value)


def load_path(path, offset=(0., 0.)):
    import numpy as np
    points, lines = [], []
    with Path(path).open(encoding='utf-8-sig') as stream:
        for line_no, line in enumerate(stream, 1):
            parts = line.split()
            if not parts:
                continue
            if len(parts) < 2:
                raise ValueError('path line %d needs at least X Y' % line_no)
            try:
                x, y = float(parts[0]), float(parts[1])
            except ValueError as exc:
                raise ValueError('path line %d has invalid X Y' % line_no) from exc
            if not math.isfinite(x) or not math.isfinite(y):
                raise ValueError('path line %d has nonfinite X Y' % line_no)
            points.append((x + offset[0], y + offset[1]))
            lines.append(line_no)
    if len(points) < 2:
        raise ValueError('path needs at least two waypoints')
    return np.asarray(points), lines


def protect_outputs(outputs, inputs):
    sources = {Path(p).resolve() for p in inputs} | {DEFAULT_PATH.resolve(), DEFAULT_CONFIG.resolve()}
    for output in outputs:
        if output is not None and Path(output).resolve() in sources:
            raise ValueError('report must not overwrite input: ' + str(output))


def write_csv(path, rows, fields):
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)


def write_json(path, data):
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open('w', encoding='utf-8') as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
