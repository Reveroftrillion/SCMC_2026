"""Offline tooling regression checks; no ROS packages or simulator required."""
import contextlib
import copy
import csv
import hashlib
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import unittest
import uuid
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'scripts'))
from local_planner_tools import DEFAULT_CONFIG, load_document, load_path, protect_outputs
from check_local_planner_config import check_config, main as preflight_main
from analyze_global_path import analyze_path, main as analyze_main
from local_planner_scenarios import scenarios, evaluate_scenario
from run_local_planner_scenarios import main as scenarios_main
from sweep_local_planner_params import variants, classify, main as sweep_main


@contextlib.contextmanager
def tool_tempdir():
    parent=ROOT/'reports/local_planner'
    parent.mkdir(parents=True,exist_ok=True)
    if ROOT.resolve() not in parent.resolve().parents:
        raise ValueError('test temp directory must stay in workspace')
    # Inherit workspace ACLs on Windows; mkdtemp(mode=0700) can exclude a sandbox identity.
    folder=parent/('tools_test_'+uuid.uuid4().hex)
    folder.mkdir()
    try:
        yield str(folder)
    finally:
        if folder.resolve().parent != parent.resolve() or ROOT.resolve() not in folder.resolve().parents:
            raise ValueError('unexpected test temp directory')
        shutil.rmtree(folder)


class OfflineToolTests(unittest.TestCase):
    def setUp(self):
        self.document = load_document(DEFAULT_CONFIG)
        self.p = self.document['static_obstacle_avoidance_planner']

    def errors(self, document=None):
        return [f for f in check_config(document or self.document) if f['level'] == 'ERROR']

    def test_default_config_warns_on_empty_missions_and_tf(self):
        results = check_config(self.document)
        self.assertFalse(self.errors())
        self.assertEqual({f['item'] for f in results if f['level'] == 'WARNING'},
                         {'static_zones','dynamic_zones','lidar_xyz_rpy'})

    def test_preflight_aggregates_geometry_horizon_offsets_and_speed_errors(self):
        self.p.update(vehicle_width=12., wheelbase=10., horizon=8., offsets=[-10.,0.,10.],
                      static_speed_kmh=-1., dynamic_speed_kmh=float('nan'), sample_step=float('inf'))
        items = {f['item'] for f in self.errors()}
        self.assertTrue({'footprint/corridor','wheelbase/length','horizon coverage','offsets/corridor',
                         'static_speed_kmh','dynamic_speed_kmh','sample_step'} <= items)

    def test_non_numeric_bool_missing_and_bad_offsets_are_errors(self):
        for key, value in [('vehicle_width',True),('vehicle_front','TODO_MEASURE'),('offsets',[]),('offsets',[0.,float('inf')])]:
            with self.subTest(key=key,value=value):
                document = copy.deepcopy(self.document)
                document['static_obstacle_avoidance_planner'][key] = value
                self.assertIn(key, {f['item'] for f in self.errors(document)})
        del self.p['return_length']
        self.assertIn('return_length', {f['item'] for f in self.errors()})

    def test_zone_reversal_malformed_and_path_bounds(self):
        for zones in ([{'start':10,'end':2}], [{'start':False,'end':2}], [{'xmin':0,'xmax':1,'ymin':2,'ymax':1}], 'bad'):
            self.p['static_zones'] = zones
            self.assertIn('static_zones', {f['item'] for f in self.errors()})
        self.p['static_zones'] = [{'start':0,'end':5}]
        results = check_config(self.document, np.array([[0.,0.],[1.,0.]]))
        self.assertTrue(any(f['level']=='ERROR' and f['item']=='static_zones[0]' for f in results))

    def test_zone_overlap_same_cross_and_mixed_representations(self):
        self.p['static_zones'] = [{'start':0,'end':2},{'start':2,'end':3}]
        self.p['dynamic_zones'] = [{'start':1,'end':2}]
        overlaps = [f for f in check_config(self.document) if f['item']=='zone overlap']
        self.assertEqual(len(overlaps),3)
        self.p['static_zones'] = [{'start':0,'end':1}]
        self.p['dynamic_zones'] = [dict(xmin=4.,xmax=6.,ymin=-1.,ymax=1.)]
        points=np.array([[0.,0.],[10.,0.],[20.,0.]])
        overlaps=[f for f in check_config(self.document,points) if f['item']=='zone overlap']
        self.assertEqual(len(overlaps),1)  # Segment crosses box although neither endpoint is inside.
        self.assertIn('overlaps',overlaps[0]['message'])
        self.assertIn('need --path',[f for f in check_config(self.document) if f['item']=='zone overlap'][0]['message'])

    def test_extrinsics_invalid_and_external_tf(self):
        for value in ([0.]*5,[0.,0.,0.,0.,0.,float('nan')],['TODO_MEASURE']*6):
            self.document['viz_planner']['lidar_xyz_rpy']=value
            self.assertIn('lidar_xyz_rpy',{f['item'] for f in self.errors()})
        self.document['viz_planner']['lidar_xyz_rpy']=[]
        results=check_config(self.document,external_lidar_tf=True)
        self.assertTrue(any(f['item']=='lidar_xyz_rpy' and f['level']=='PASS' for f in results))
        self.document['viz_planner']['lidar_xyz_rpy']=[0.]*6
        self.assertTrue(any(f['item']=='lidar_xyz_rpy' and f['level']=='WARNING'
                            for f in check_config(self.document,external_lidar_tf=True)))

    def test_test_template_has_same_keys_and_measurements_are_blocked(self):
        document=load_document(ROOT/'src/planning/config/local_planner_test.yaml')
        self.assertEqual(set(document['static_obstacle_avoidance_planner']),set(self.p))
        items={f['item'] for f in self.errors(document)}
        self.assertTrue({'vehicle_width','vehicle_front','vehicle_rear','wheelbase','road_half_width','lidar_xyz_rpy'} <= items)

    def test_yaml_duplicate_keys_and_bad_path_rows_rejected(self):
        with tool_tempdir() as folder:
            file=Path(folder)/'input.txt'
            file.write_text('horizon: 26\nhorizon: 8\n',encoding='utf-8')
            with self.assertRaisesRegex(ValueError,'duplicate YAML key'): load_document(file)
            for text in ('0 0\n1 nan\n','0 0\ninvalid 1\n','0 0\n1\n'):
                file.write_text(text,encoding='utf-8')
                with self.assertRaises(ValueError): load_path(file)

    def test_path_spacing_duplicates_yaw_closure_and_no_mutation(self):
        points=np.array([[0.,0.],[1.,0.],[1.,0.],[1.001,0.],[0.,0.]])
        original=points.copy()
        summary,rows=analyze_path(points,self.p,check_references=False)
        self.assertEqual(summary['waypoint_count'],5)
        self.assertEqual(summary['consecutive_duplicate_segment_count'],1)
        self.assertEqual(summary['near_duplicate_segment_count'],1)
        self.assertEqual(summary['yaw_discontinuity_indices'],[3])
        self.assertTrue(summary['path_closed'])
        self.assertEqual(len(rows),4)
        np.testing.assert_array_equal(points,original)

    def test_constant_radius_discrete_curvature_and_spikes(self):
        theta=np.linspace(0.,1.,51)
        points=np.column_stack((10.*np.sin(theta),10.*(1-np.cos(theta))))
        p=copy.deepcopy(self.p)
        p['max_reference_curvature']=.09
        summary,_=analyze_path(points,p,check_references=False)
        self.assertAlmostEqual(summary['max_discrete_curvature_per_m'],.1,places=8)
        self.assertEqual(len(summary['curvature_spike_indices']),49)

    def test_runtime_reference_risk_contains_path_end_and_sharp_turn(self):
        points=np.column_stack((np.arange(0.,40.5,.5),np.zeros(81)))
        summary,_=analyze_path(points,self.p)
        self.assertIn(80,{r['index'] for r in summary['local_spline_risks']})
        # A short path cannot fit transition/return and must report every index.
        summary,_=analyze_path(np.array([[0.,0.],[1.,0.],[2.,0.],[1.,0.]]),self.p)
        self.assertEqual(len(summary['local_spline_risks']),4)
        self.assertTrue(any('yaw discontinuity' in r['reason'] for r in summary['local_spline_risks']))

    def test_all_scenarios_and_temporal_expectations(self):
        fixtures=scenarios()
        self.assertEqual(len(fixtures),12)
        rows=[r for s in fixtures for r in evaluate_scenario(s,self.p)]
        self.assertEqual(len(rows),18)
        self.assertTrue(all(r['result']=='PASS' for r in rows),rows)
        disappearance=[r for r in rows if r['scenario']=='obstacle_disappears']
        self.assertEqual([r['memory_obstacles'] for r in disappearance],[1,1,0,0])
        self.assertEqual([r['state'] for r in disappearance],['STATIC_OBSTACLE','STATIC_OBSTACLE','RETURN_TO_GLOBAL','NORMAL'])

    def test_scenario_failure_is_not_silently_passed(self):
        fixture=scenarios()[1]
        fixture.frames[0].expected_state='NORMAL'
        self.assertEqual(evaluate_scenario(fixture,self.p)[0]['result'],'FAIL')
        bad=copy.deepcopy(self.p); bad['horizon']=1.
        self.assertEqual(evaluate_scenario(fixture,bad)[0]['result'],'CONFIG_ERROR')

    def test_sweep_counts_grid_cap_and_original_config_unchanged(self):
        before=copy.deepcopy(self.p)
        grid={'safety_margin':[.1,.5],'offsets':[[-2.,0.,2.],[-3.,0.,3.]]}
        self.assertEqual(len(list(variants(self.p,grid))),5)
        self.assertEqual(len(list(variants(self.p,grid,'cartesian'))),5)
        self.assertEqual(before,self.p)
        with self.assertRaisesRegex(ValueError,'exceed'): list(variants(self.p,grid,'cartesian',4))
        with self.assertRaises(ValueError): list(variants(self.p,{'offsets':[1.,2.]}))
        with self.assertRaises(ValueError): list(variants(self.p,{'unknown':[1.]}))

    def test_sweep_reports_blocked_low_clearance_and_config_error(self):
        fixtures=scenarios()
        centre=evaluate_scenario(fixtures[1],self.p)[0]
        self.assertIn('LOW_CLEARANCE',classify(centre,self.p,.25))
        blocked=evaluate_scenario(fixtures[5],self.p)[0]
        self.assertEqual(classify(blocked,self.p,.25),'ALL_REJECTED')
        p=copy.deepcopy(self.p); p['horizon']=1.
        row=evaluate_scenario(fixtures[1],p)[0]
        self.assertEqual(classify(row,p,.25),'CONFIG_ERROR')

    def test_cli_reports_and_overwrite_protection(self):
        with tool_tempdir() as folder, contextlib.redirect_stdout(io.StringIO()):
            root=Path(folder)
            config=root/'config.yaml'; config.write_text(yaml.safe_dump(self.document),encoding='utf-8')
            path=root/'path.txt'; path.write_text('\n'.join('%s 0'%x for x in range(60)),encoding='utf-8')
            digest=hashlib.sha256(path.read_bytes()).hexdigest()
            self.assertEqual(preflight_main(['--config',str(config),'--json',str(root/'preflight.json')]),0)
            self.assertEqual(json.loads((root/'preflight.json').read_text())['errors'],0)
            self.assertEqual(analyze_main(['--config',str(config),'--path',str(path),'--csv',str(root/'path.csv')]),0)
            self.assertTrue((root/'path.summary.json').exists())
            self.assertEqual(scenarios_main(['--config',str(config),'--csv',str(root/'scenario.csv')]),0)
            grid=root/'grid.yaml'; grid.write_text('horizon: [1, 26]\n',encoding='utf-8')
            self.assertEqual(sweep_main(['--config',str(config),'--grid',str(grid),'--scenario','center_obstacle','--csv',str(root/'sweep.csv')]),0)
            with (root/'sweep.csv').open(newline='') as stream:
                self.assertIn('CONFIG_ERROR',{r['assessment'] for r in csv.DictReader(stream)})
            self.assertEqual(analyze_main(['--path',str(path),'--csv',str(path)]),2)
            self.assertEqual(preflight_main(['--config',str(config),'--json',str(config)]),2)
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),digest)
            with self.assertRaises(ValueError): protect_outputs([path],[path])


@unittest.skipUnless(os.name=='posix' and shutil.which('bash') and shutil.which('timeout'),
                     'diagnostic collector execution requires POSIX Bash/GNU timeout')
class DiagnosticCollectorTests(unittest.TestCase):
    def run_collector(self, fake_ros=False):
        with tool_tempdir() as folder:
            base=Path(folder); tools=base/'bin'; tools.mkdir()
            for name in ('dirname','date','mkdir','mktemp','timeout','grep','cat','rm','cp','basename','sha256sum','git','sleep'):
                command=shutil.which(name)
                if command: (tools/name).symlink_to(command)
            if fake_ros:
                for name, body in {
                    'rostopic':'case "$1" in hz) echo "average rate: 10.0"; sleep 5;; list) echo /local_plan;; echo) echo "state: NORMAL";; esac',
                    'rosrun':'echo "Translation: [0, 0, 0]"; sleep 5',
                    'rosparam':'echo "ERROR: no parameter" >&2; exit 1',
                }.items():
                    file=tools/name; file.write_text('#!/bin/bash\n'+body+'\n'); file.chmod(0o755)
            env=dict(os.environ,PATH=str(tools))
            result=subprocess.run([shutil.which('bash'),str(ROOT/'scripts/collect_local_planner_debug.sh'),
                                   '--timeout','1','--output-root',str(base/'logs')],env=env,
                                  capture_output=True,text=True,timeout=20)
            self.assertEqual(result.returncode,0,result.stderr)
            folder=next((base/'logs').iterdir())
            self.assertTrue((folder/'local_planner.yaml').exists())
            self.assertTrue((folder/'manifest.txt').exists())
            self.assertIn('unavailable',(folder/'params_local.txt').read_text())
            self.assertIn('git_commit.txt',(folder/'summary.tsv').read_text())
            if fake_ros:
                self.assertIn('PASS (bounded sample',(folder/'hz_local_plan.txt').read_text())
                self.assertIn('PASS (bounded sample',(folder/'tf_map_lidar.txt').read_text())
                self.assertIn('state: NORMAL',(folder/'local_plan_sample.txt').read_text())
            else:
                self.assertIn('unavailable',(folder/'local_plan_sample.txt').read_text())

    def test_missing_ros_does_not_abort_collection(self):
        self.run_collector()

    def test_bounded_samples_and_failed_params_are_independent(self):
        self.run_collector(fake_ros=True)
