#!/usr/bin/env python3
"""Fetch pinned ROS release archives and verify SHA256 before extraction.

Avoid legacy wstool's yaml.load/PyYAML 6 incompatibility without changing ROS
sources or the numerical runtime. --record is an explicit lock creation step.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import shutil
import tarfile
import tempfile
import time
from urllib.request import urlopen

import yaml


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('manifest', type=Path)
    p.add_argument('hashes', type=Path)
    p.add_argument('destination', type=Path)
    p.add_argument('--record', action='store_true')
    args = p.parse_args()
    if args.record and args.hashes.exists():
        raise SystemExit('Refusing to replace existing source lock')
    expected = {} if args.record else json.loads(args.hashes.read_text())
    entries = yaml.safe_load(args.manifest.read_text())
    args.destination.mkdir(parents=True, exist_ok=True)

    def fetch(entry):
        info = entry['tar']
        name, url = info['local-name'], info['uri']
        for attempt in range(4):
            try:
                with urlopen(url, timeout=90) as response:
                    data = response.read()
                break
            except Exception:
                if attempt == 3:
                    raise
                time.sleep(1 + attempt)
        checksum = hashlib.sha256(data).hexdigest()
        item = dict(uri=url, sha256=checksum, bytes=len(data))
        if not args.record and expected.get(name) != item:
            raise RuntimeError('Release archive lock mismatch: ' + name)
        with tempfile.TemporaryDirectory() as tmp:
            archive = Path(tmp) / 'release.tar.gz'
            archive.write_bytes(data)
            extracted = Path(tmp) / 'extracted'
            extracted.mkdir()
            with tarfile.open(archive) as stream:
                for member in stream.getmembers():
                    path = (extracted / member.name).resolve()
                    if not path.is_relative_to(extracted):
                        raise RuntimeError('Unsafe archive path: ' + member.name)
                    if member.issym() or member.islnk():
                        linked = (path.parent / member.linkname).resolve()
                        if not linked.is_relative_to(extracted):
                            raise RuntimeError('Unsafe archive link: ' + member.name)
                stream.extractall(extracted)
            roots = list(extracted.iterdir())
            if len(roots) != 1 or not roots[0].is_dir():
                raise RuntimeError('Unexpected archive root: ' + name)
            target = args.destination / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.move(str(roots[0]), str(target))
        print(name, checksum, flush=True)
        return name, item

    with ThreadPoolExecutor(max_workers=4) as pool:
        actual = dict(pool.map(fetch, entries))
    if args.record:
        args.hashes.write_text(json.dumps(actual, indent=2, sort_keys=True) + '\n')
    elif actual != expected:
        raise SystemExit('Release package set differs from lock')


if __name__ == '__main__':
    main()
