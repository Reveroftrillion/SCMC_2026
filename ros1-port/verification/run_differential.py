#!/usr/bin/env python3
"""Run the unmodified upstream tests twice on disposable source copies.

This deliberately does not simulate real ROS transport/scheduling. See
ros_stubs.py and runtime/test_adapter.py for the distinct verification scopes.
"""
import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
import importlib.metadata
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from ctrl_names import ctrl_text, legacy_text, current_relative

HERE=Path(__file__).resolve().parent
PORT=HERE.parent
REPO=PORT.parent
PACKAGES=('ctrl_control','ctrl_planning','ctrl_lidar','ctrl_camera')
ATOL=RTOL=1e-12

def compare(a,b,path='',differences=None,stats=None):
    differences=[] if differences is None else differences
    stats={'numbers':0,'max_abs_error':0.} if stats is None else stats
    if isinstance(a,int) and isinstance(b,int):
        stats['numbers']+=1; stats['max_abs_error']=max(stats['max_abs_error'],abs(a-b))
        if a!=b: differences.append(path)
    elif isinstance(a,(int,float)) and not isinstance(a,bool) and isinstance(b,(int,float)) and not isinstance(b,bool):
        stats['numbers']+=1; stats['max_abs_error']=max(stats['max_abs_error'],abs(a-b))
        if not math.isclose(a,b,abs_tol=ATOL,rel_tol=RTOL): differences.append(path)
    elif isinstance(a,dict) and isinstance(b,dict):
        if a.keys()!=b.keys(): differences.append(path+':keys')
        for key in a.keys()&b.keys(): compare(a[key],b[key],path+'/'+key,differences,stats)
    elif isinstance(a,list) and isinstance(b,list):
        if len(a)!=len(b): differences.append(path+':length')
        for i,(x,y) in enumerate(zip(a,b)): compare(x,y,path+'/'+str(i),differences,stats)
    elif a!=b: differences.append(path)
    return differences,stats

def copy_snapshot(side):
    destination=HERE/'.work'/side
    destination.mkdir(parents=True,exist_ok=True)
    source=REPO/'2026_CTRL_competition' if side=='original' else PORT
    shutil.copytree(source/'src',destination/'src',dirs_exist_ok=True,
        ignore=shutil.ignore_patterns('__pycache__','.pytest_cache','*.pyc'))
    # Original test_highway_measurement imports these offline helpers.
    shutil.copytree(REPO/'2026_CTRL_competition/tools',destination/'tools',dirs_exist_ok=True)
    if side=='port':
        for helper in (destination/'tools').rglob('*.py'):
            helper.write_text(ctrl_text(helper.read_text(encoding='utf-8')),encoding='utf-8')
    return destination

def run(side, selected=None, suffix=''):
    tree=copy_snapshot(side)
    report=HERE/'.work'/('report-'+side+suffix+'.json')
    if report.exists(): report.unlink()
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONHASHSEED='0',PYTHONUTF8='1',
        PYTEST_DISABLE_PLUGIN_AUTOLOAD='1',FOSCAR_TEST_TREE=str(tree),FOSCAR_TEST_REPORT=str(report),
        PYTHONPATH=str(HERE),OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
    # Explicit sys.path also supports isolated Windows embeddable Python3.10,
    # whose ._pth correctly ignores PYTHONPATH.
    bootstrap='import sys; sys.path.insert(0,'+repr(str(HERE))+'); import pytest; raise SystemExit(pytest.main(sys.argv[1:]))'
    command=[sys.executable,'-X','utf8','-c',bootstrap,'-p','pytest_bridge','-p','no:cacheprovider','-q',
        '--rootdir='+str(tree),'--basetemp='+str(HERE/'.work'/('tmp-'+side))]
    if selected:
        for item in selected:
            relative=item.split('::',1)[0]
            relative=legacy_text(relative) if side=='original' else current_relative(relative)
            command.append(str(tree/relative)+('::'+item.split('::',1)[1] if '::' in item else ''))
    else:
        command.extend(str(tree/'src'/(legacy_text(p) if side=='original' else p)/'test') for p in PACKAGES)
    log=HERE/'.work'/('pytest-'+side+suffix+'.log')
    with log.open('w',encoding='utf-8') as stream:
        result=subprocess.run(command,cwd=tree,env=env,stdout=stream,stderr=subprocess.STDOUT)
    print(side+': exit '+str(result.returncode)+'; '+str(log),flush=True)
    if not report.exists(): raise RuntimeError('No report; inspect '+str(log))
    return json.loads(report.read_text(encoding='utf-8'))

def summarize(report):
    statuses=Counter(r['outcome'] for r in report['reports'])
    failed=[dict(id=r['id'],phase=r['phase'],details=r['details']) for r in report['reports'] if r['outcome']=='failed']
    return dict(exitstatus=report['exitstatus'],statuses=dict(statuses),failed=failed,
        trace_count=len(report['traces']),operations=dict(Counter(r['operation'] for r in report['traces'])))

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reuse-reports',action='store_true')
    parser.add_argument('--rerun-failed',action='store_true',help='rerun failed cases only and retain full-run provenance')
    args=parser.parse_args()
    if args.reuse_reports or args.rerun_failed:
        a,b=[json.loads((HERE/'.work'/('report-'+side+'.json')).read_text(encoding='utf-8')) for side in ('original','port')]
    else:
        with ThreadPoolExecutor(max_workers=2) as pool:
            a,b=list(pool.map(run,('original','port')))
    if args.rerun_failed:
        selected=sorted({r['id'] for report in (a,b) for r in report['reports'] if r['outcome']=='failed'})
        if not selected: raise SystemExit('No failed cases to rerun')
        with ThreadPoolExecutor(max_workers=2) as pool:
            again=list(pool.map(lambda side:run(side,selected,'-rerun'),('original','port')))
        for side,report,replayed in zip(('original','port'),(a,b),again):
            previous=dict(exitstatus=report['exitstatus'],failed=[r for r in report['reports'] if r['outcome']=='failed'])
            order={r['id']:i for i,r in enumerate(report['reports'])}
            report['reports']=[r for r in report['reports'] if r['id'] not in selected]+replayed['reports']
            report['reports'].sort(key=lambda r:order[r['id']])
            report['traces']=[r for r in report['traces'] if r['test'] not in selected]+replayed['traces']
            report['traces'].sort(key=lambda r:order.get(r['test'],-1))
            report['exitstatus']=1 if any(r['outcome']=='failed' for r in report['reports']) else 0
            report['rerun']=dict(previous=previous,selected=selected,exitstatus=replayed['exitstatus'])
            (HERE/'.work'/('report-'+side+'.json')).write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf-8')
    statuses_a=[(r['id'],r['phase'],r['outcome']) for r in a['reports']]
    statuses_b=[(r['id'],r['phase'],r['outcome']) for r in b['reports']]
    differences,stats=compare(a['traces'],b['traces'])
    cases=dict(original=statuses_a,port=statuses_b)
    (HERE/'case-outcomes.json').write_text(json.dumps(cases,indent=2,ensure_ascii=False),encoding='utf-8')
    summary=dict(scope='Windows host-only algorithm/callback differential with common test doubles; not ROS integration',
        python=sys.version,versions={p:importlib.metadata.version(p) for p in ('pytest','numpy','scipy','PyYAML','pyproj','scikit-learn','matplotlib')},
        absolute_tolerance=ATOL,relative_tolerance=RTOL,original=summarize(a),port=summarize(b),
        identical_case_outcomes=statuses_a==statuses_b,trace_differences=differences[:100],numeric_comparison=stats)
    summary['integer_timestamp_boolean_state_comparison']='exact'
    for distribution in ('opencv-python','opencv-python-headless'):
        try:
            summary['versions'][distribution]=importlib.metadata.version(distribution)
            break
        except importlib.metadata.PackageNotFoundError: pass
    summary['all_tests_passed']=a['exitstatus']==b['exitstatus']==0
    summary['rerun_provenance']={side:report.get('rerun') for side,report in [('original',a),('port',b)]}
    summary['trace_sha256']={side:hashlib.sha256(json.dumps(report['traces'],sort_keys=True,
        separators=(',',':'),ensure_ascii=False).encode('utf-8')).hexdigest() for side,report in [('original',a),('port',b)]}
    target=HERE/'differential-results.json'
    target.write_text(json.dumps(summary,indent=2,ensure_ascii=False),encoding='utf-8')
    print(json.dumps({k:v for k,v in summary.items() if k not in ('original','port','rerun_provenance')},indent=2))
    print('original:',summary['original']['statuses'],'port:',summary['port']['statuses'])
    print('Results:',target)
    if differences or statuses_a!=statuses_b or not a['reports'] or not a['traces']: return 1
    if a['exitstatus'] or b['exitstatus']: return 2
    return 0

if __name__=='__main__': raise SystemExit(main())
