#!/usr/bin/env python3
"""Summarize an ended audit, re-decoding collision wire bytes without ROS.

Never edit the input evidence; require a new output file. This reports MORAI
telemetry and candidate box observations, not official competition scoring.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

sys.dont_write_bytecode = True
PORT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PORT/'docker'))
from course_audit_safety import collision_slots, decode_collision


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def collision_report(path):
    packets, hits, errors = 0, [], []
    with path.open() as stream:
        for line_number, line in enumerate(stream, 1):
            row = json.loads(line)
            if row.get('port') != 9092:
                continue
            packets += 1
            try:
                slots = decode_collision(bytes.fromhex(row['raw_hex']))
            except (ValueError, KeyError, TypeError) as error:
                errors.append(dict(line=line_number, error=str(error)))
                continue
            colliding = collision_slots(slots)
            if colliding:
                hits.append(dict(time=row['time'], slots=colliding))
    return dict(packets=packets, collision_packets=len(hits), collisions=hits,
                parse_errors=errors, wire_sha256=sha256(path))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--analysis', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if list(args.run.rglob('*.active')):
        parser.error('Active bags must be closed')
    end = json.loads((args.run/'end.json').read_text())
    analysis = json.loads((args.analysis/'summary.json').read_text())
    collision = collision_report(args.run/'wire.jsonl')
    evidence = [args.run/'end.json', args.run/'source-before.json', args.analysis/'summary.json']
    result = dict(scope='Ended native MORAI/ROS1 audit with raw collision re-decode; no official score',
                  run=str(args.run), termination=end, tunnel=analysis, collision=collision,
                  evidence_sha256={str(p): sha256(p) for p in evidence},
                  bags=[dict(name=p.name, bytes=p.stat().st_size, sha256=sha256(p))
                        for p in sorted(args.run.glob('*.bag'))])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(dict(reason=end['reason'], matched_observations=analysis['matched_observations'],
        collision_packets=collision['collision_packets'], parse_errors=len(collision['parse_errors']),
        handoff=bool(end.get('manual_park_stationary_evidence')), output=str(args.output))))


if __name__ == '__main__':
    main()
