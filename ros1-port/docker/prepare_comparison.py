#!/usr/bin/env python3
"""Build a disposable comparison tree; never change original checkout bytes.

The recorded Windows manifest requires CRLF. Reconstruct only when the exact
existing SHA256 verifies; unknown content differences abort. Record every case.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

PORT = Path(__file__).resolve().parents[1]
REPO = PORT.parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    target = args.destination.resolve()
    if target.exists():
        raise SystemExit('Destination already exists: ' + str(target))
    manifest = json.loads((PORT / 'verification/source-manifest.json').read_text())['files']
    records = []
    for name, entry in manifest.items():
        path = REPO / '2026_CTRL_competition' / name
        data = path.read_bytes()
        before = hashlib.sha256(data).hexdigest()
        action = 'exact'
        if before != entry['sha256']:
            data = data.replace(b'\r\n', b'\n').replace(b'\n', b'\r\n')
            action = 'restore_recorded_CRLF_in_disposable_copy'
        if len(data) != entry['bytes'] or hashlib.sha256(data).hexdigest() != entry['sha256']:
            raise SystemExit('Content mismatch beyond line endings; refusing reconstruction: ' + name)
        dest = target / '2026_CTRL_competition' / name
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_bytes(data)
        records.append(dict(path=name, checkout_sha256=before, manifest_sha256=entry['sha256'], action=action))
    shutil.copytree(PORT, target / 'ros1-port', ignore=shutil.ignore_patterns(
        'build', 'devel', 'install', 'artifacts', '.work', '__pycache__', '.pytest_cache'))
    (target / 'checkout-line-endings.json').write_text(json.dumps(records, indent=2) + '\n')
    print('Prepared', len(records), 'verified original files;', sum(r['action'] != 'exact' for r in records),
          'CRLF reconstructions in disposable copy only:', target)


if __name__ == '__main__':
    main()
