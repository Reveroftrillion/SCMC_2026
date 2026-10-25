#!/usr/bin/env python3
"""Verify immutable baseline hashes and exact reviewed ROS boundary transform."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from ctrl_names import current_path

HERE=Path(__file__).resolve().parent
PORT=HERE.parent
REPO=PORT.parent
SOURCE=REPO/'2026_CTRL_competition'
MANIFEST=HERE/'source-manifest.json'

def digest(path): return hashlib.sha256(path.read_bytes()).hexdigest()

def inventory():
    return {p.relative_to(SOURCE).as_posix():dict(bytes=p.stat().st_size,sha256=digest(p))
        for p in sorted(SOURCE.rglob('*')) if p.is_file() and
        not any(part in ('__pycache__','.pytest_cache') for part in p.parts)}

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record-baseline',action='store_true')
    args=parser.parse_args()
    actual=inventory()
    if args.record_baseline:
        if MANIFEST.exists(): raise SystemExit('Refusing to overwrite existing baseline manifest')
        revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip()
        MANIFEST.write_text(json.dumps(dict(git_head=revision,reference='2026_CTRL_competition',files=actual),
            indent=2,ensure_ascii=False),encoding='utf-8')
        print('Recorded',len(actual),'original files')
    recorded=json.loads(MANIFEST.read_text(encoding='utf-8'))['files']
    if actual!=recorded: raise SystemExit('Reference changed since baseline snapshot')
    spec=importlib.util.spec_from_file_location('port_boundary',PORT/'tools/port_boundary.py')
    boundary=importlib.util.module_from_spec(spec);spec.loader.exec_module(boundary)
    pure=[];adapted=[];errors=[]
    for path in sorted((SOURCE/'src').glob('*/*/**/*.py')):
        rel=path.relative_to(SOURCE/'src')
        if len(rel.parts)<3 or rel.parts[0]!=rel.parts[1]: continue
        target=current_path(PORT/'src'/rel)
        original=path.read_text(encoding='utf-8')
        expected=boundary.converted(rel,original)
        if not target.exists() or target.read_text(encoding='utf-8')!=expected: errors.append(str(rel))
        (pure if expected==original else adapted).append(rel.as_posix())
        if expected==original and target.exists() and target.read_bytes()!=path.read_bytes(): errors.append(str(rel)+':bytes')
    # Existing tests are fixtures, retained byte-for-byte.
    tests=list((SOURCE/'src').glob('*/test/**/*'))
    test_count=0
    for path in tests:
        if not path.is_file(): continue
        rel=path.relative_to(SOURCE/'src');target=current_path(PORT/'src'/rel);test_count+=1
        if not target.exists() or digest(path)!=digest(target): errors.append(str(rel))
    report=dict(reference_files=len(actual),unchanged_python=len(pure),boundary_python=len(adapted),
        unchanged_test_files=test_count,adapted_files=adapted,errors=errors)
    (HERE/'preservation-results.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))
    if errors: raise SystemExit(1)

if __name__=='__main__': main()
