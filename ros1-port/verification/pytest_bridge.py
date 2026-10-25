"""Pytest plugin installed only by run_differential.py."""
import json
import os
from pathlib import Path
import sys
import dataclasses
import functools
import importlib
import math
import ros_stubs
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from ctrl_names import legacy_text, module_name

TREE = Path(os.environ['FOSCAR_TEST_TREE'])
for package in (TREE/'src').iterdir():
    if package.is_dir(): sys.path.insert(0,str(package))
ros_stubs.install(TREE)
REPORTS=[]
TRACES=[]
CURRENT='collection'

def normalized(value):
    import numpy as np
    if isinstance(value, np.ndarray): return normalized(value.tolist())
    if isinstance(value, np.generic): return normalized(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return 'NaN' if math.isnan(value) else '+Inf' if value>0 else '-Inf'
    if value is None or isinstance(value,(bool,int,float)): return value
    if isinstance(value,str): return legacy_text(value.replace(str(TREE),'<TREE>'))
    if dataclasses.is_dataclass(value): return {f.name:normalized(getattr(value,f.name)) for f in dataclasses.fields(value)}
    if isinstance(value,dict): return {str(k):normalized(v) for k,v in value.items()}
    if isinstance(value,(tuple,list)): return [normalized(v) for v in value]
    if isinstance(value,(set,frozenset)): return sorted(normalized(v) for v in value)
    if type(value).__name__=='ReferenceLine':
        return {k:normalized(getattr(value,k)) for k in ('id','global_s','xy','q','signature') if hasattr(value,k)}
    if isinstance(value,ros_stubs.Message): return {k:normalized(v) for k,v in vars(value).items()}
    return '<'+type(value).__name__+'>'

def pytest_sessionstart(session):
    targets=[('ctrl_planning.ctrl_state','Behavior','tick'),
        ('ctrl_planning.ctrl_signal_stop','SignalStopPolicy','apply'),
        ('ctrl_planning.ctrl_signal_stop','SignalStopPolicy','observe'),
        ('ctrl_planning.states.ctrl_avoid','Avoid','compute_target'),
        ('ctrl_planning.states.ctrl_yield','Yield','compute_target'),
        ('ctrl_planning.ctrl_circular_gap','CircularGap','evaluate'),
        ('ctrl_planning.ctrl_highway_runtime','HighwayRuntime','apply'),
        ('ctrl_control.ctrl_core','Controller','step'),
        ('ctrl_control.ctrl_core','PoseEstimator','update')]
    for module,cls,method in targets:
        if TREE.name=='original':module=legacy_text(module)
        owner=getattr(importlib.import_module(module),cls)
        original=getattr(owner,method)
        label=cls+'.'+method
        def wrapper(self,*args,_original=original,_label=label,**kwargs):
            inputs=dict(args=normalized(args),kwargs=normalized(kwargs))
            result=_original(self,*args,**kwargs)
            row=dict(test=CURRENT,operation=_label,inputs=inputs,result=normalized(result))
            if _label=='Behavior.tick': row['engaged']=normalized(self.engaged)
            if _label=='PoseEstimator.update':
                row['pose']={k:normalized(getattr(self,k,None)) for k in ('x','y','mode','dr_distance','last_error')}
            if _label.startswith('SignalStopPolicy.'):
                row['memory']={k:normalized(getattr(self,k)) for k in ('key','color','last_s','last_stamp')}
            TRACES.append(row)
            return result
        setattr(owner,method,functools.wraps(original)(wrapper))
    for modname,names in [('mission_markers',('mission_markers','checkpoint_markers','required_checkpoint_markers','bev_stop_markers')),
            ('prediction_markers',('prediction_markers',)),('roundabout_markers',('roundabout_markers',)),
            ('highway_markers',('highway_markers',))]:
        full='foscar_planning.'+modname
        owner=importlib.import_module(full if TREE.name=='original' else module_name(full))
        for name in names:
            original=getattr(owner,name)
            def wrapped(*args,_original=original,_label=name,**kwargs):
                result=_original(*args,**kwargs)
                TRACES.append(dict(test=CURRENT,operation=_label,result=normalized(result)))
                return result
            setattr(owner,name,functools.wraps(original)(wrapped))

def pytest_runtest_setup(item):
    global CURRENT
    CURRENT=legacy_text(item.nodeid.replace('\\','/'))

def pytest_runtest_logreport(report):
    if report.when=='call' or (report.when in ('setup','teardown') and report.failed):
        REPORTS.append(dict(id=legacy_text(report.nodeid.replace('\\','/')),phase=report.when,outcome=report.outcome,
            details=str(report.longrepr) if report.failed else ''))

def pytest_sessionfinish(session,exitstatus):
    Path(os.environ['FOSCAR_TEST_REPORT']).write_text(json.dumps(dict(exitstatus=int(exitstatus),
        reports=REPORTS,traces=TRACES),indent=2,ensure_ascii=False),encoding='utf-8')
