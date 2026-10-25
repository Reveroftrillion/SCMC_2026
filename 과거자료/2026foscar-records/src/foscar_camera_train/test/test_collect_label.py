"""One-shot collection: real child processes, synthetic photos, no MORAI commands."""

import csv
import json
import os
from pathlib import Path
import pty
import signal
import subprocess
import sys
import time

import cv2
import pytest

from foscar_camera_train import extract as ex
from synth import four


def make_session(path, positions):
    for folder in ('rgb', 'sem'):
        (path / folder).mkdir(parents=True)
    with (path / 'meta.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['idx', 's'])
        for idx, position in enumerate(positions):
            rgb, sem = four(**({'red': 'red'} if idx % 2 else {'green': 'green'}))
            cv2.imwrite(str(path / 'rgb' / f'{idx:06d}.jpg'), rgb)
            cv2.imwrite(str(path / 'sem' / f'{idx:06d}.jpg'), sem)
            writer.writerow([idx, position])
    return path


def test_require_s_excludes_missing_and_nonfinite_before_dedup(tmp_path):
    pytest.importorskip('ament_index_python')
    session = make_session(tmp_path / '20260922_120000_foggy_13h', ['', 153.5, 'nan', 'inf', 879.3])
    assert ex.main(['--session', str(session), '--out', str(tmp_path / 'out'),
                    '--require-s', '--no-dedup']) == 0
    frames = ex.load_frames(tmp_path / 'out/frames.json')
    assert [f.idx for f in frames] == [1, 4]
    assert len(list((session / 'rgb').glob('*.jpg'))) == 5


def test_require_s_does_not_generate_empty_dataset(tmp_path):
    session = make_session(tmp_path / '20260922_120000_foggy_13h', ['', 'nan'])
    assert ex.main(['--session', str(session), '--out', str(tmp_path / 'out'), '--require-s']) == 1
    assert not (tmp_path / 'out/dataset/data.yaml').exists()


def wait_for_file(path, process):
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline and not path.exists():
        if process.poll() is not None:
            pytest.fail(f'process exited early: {process.communicate()[0]}')
        time.sleep(.02)
    assert path.exists(), f'Timeout waiting for {path}'


@pytest.mark.parametrize('mode', ['finish', 'terminate', 'record_failure', 'missing_s', 'node_failure'])
def test_one_shot_waits_for_flush_and_labels_only_new_session(tmp_path, mode):
    pytest.importorskip('ament_index_python')
    fixture = make_session(tmp_path / 'fixture',
                           ['', '', ''] if mode == 'missing_s' else ['', 153.5, 879.3])
    data_root, output_root = tmp_path / 'raw photos', tmp_path / 'output labels'
    old = make_session(data_root / '20260921_120000_foggy_13h', [20])
    old_csv = (old / 'meta.csv').read_bytes()
    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir()
    fake_ros = bin_dir / 'ros2'
    fake_ros.write_text('''#!/usr/bin/env python3
import os, pathlib, shutil, signal, sys, time
assert sys.argv[1:5] == ['launch', '--noninteractive', 'foscar_camera_train', 'record.launch.py']
args = dict(arg.split(':=', 1) for arg in sys.argv[5:])
assert args['tag'] == 'foggy_13h' and args['drive'] == 'false'
assert args['stop_seconds'] == '60.0'
pathlib.Path(os.environ['STARTED']).touch()
if os.environ['RECORD_FAILS'] == '1':
    sys.exit(7)
session = pathlib.Path(args['out_dir']) / '20260922_120000_foggy_13h'
shutil.copytree(os.environ['PHOTO_FIXTURE'], session)
meta = session / 'meta.csv'
rows = meta.read_text().splitlines(keepends=True)
meta.write_text(''.join(rows[:-1]))
def stop(signum, frame):
    time.sleep(.15)
    with meta.open('a') as f:
        f.write(rows[-1])
    pathlib.Path(os.environ['STOPPED']).touch()
    sys.exit(0)
signal.signal(signal.SIGINT, stop)
if os.environ['NODE_FAILS'] == '1':
    pathlib.Path(args['failure_file']).write_text('recorder_node-6 exited with code 1')
pathlib.Path(os.environ['READY']).touch()
while True:
    time.sleep(.02)
''')
    fake_ros.chmod(0o755)
    ready, stopped = tmp_path / 'ready', tmp_path / 'stopped'
    env = dict(os.environ, PATH=str(bin_dir) + os.pathsep + os.environ['PATH'],
               PHOTO_FIXTURE=str(fixture), READY=str(ready), STOPPED=str(stopped),
               STARTED=str(tmp_path / 'started'), RECORD_FAILS=str(int(mode == 'record_failure')),
               NODE_FAILS=str(int(mode == 'node_failure')))
    source = str(Path(__file__).resolve().parents[1])
    env['PYTHONPATH'] = source + os.pathsep + env.get('PYTHONPATH', '')
    command = [sys.executable, '-u', '-m', 'foscar_camera_train.collect_label', 'foggy_13h',
               '--no-drive', '--data-root', str(data_root), '--out-root', str(output_root)]
    process = subprocess.Popen(command,
        env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        start_new_session=True)
    try:
        if mode != 'record_failure':
            wait_for_file(ready, process)
            if mode == 'finish':
                concurrent = subprocess.run(command, env=env, capture_output=True, text=True, timeout=5)
                assert concurrent.returncode == 1
                assert '이미 실행 중' in concurrent.stderr
            process.send_signal(signal.SIGTERM if mode == 'terminate' else signal.SIGINT)
        output, _ = process.communicate(timeout=20)
        assert (tmp_path / 'started').exists(), output
        if mode in ('record_failure', 'terminate', 'node_failure'):
            assert process.returncode != 0, output
            assert not list(output_root.glob('*/dataset/data.yaml'))
            if mode == 'record_failure':
                assert '수집 실패' in output
            elif mode == 'node_failure':
                assert 'recorder_node-6' in output
            else:
                assert stopped.exists()
        elif mode == 'missing_s':
            assert process.returncode == 1, output
            assert not list(output_root.glob('*/dataset/data.yaml'))
            summary = json.loads(next(output_root.glob('*/summary.json')).read_text())
            assert summary['status'] == 'incomplete'
        else:
            assert process.returncode == 0, output
            assert stopped.exists()
            results = list(output_root.glob('*/frames.json'))
            assert len(results) == 1, output
            frames = ex.load_frames(results[0])
            assert [f.idx for f in frames] == [1, 2]
            assert {f.tag for f in frames} == {'foggy_13h'}
            assert all('20260922_120000' in f.rgb for f in frames)
            summary = json.loads((results[0].parent / 'summary.json').read_text())
            assert summary['status'] == 'labels_generated'
            assert summary['frames'] == 2
        assert (old / 'meta.csv').read_bytes() == old_csv
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            process.communicate(timeout=5)


@pytest.mark.parametrize('tag', ['../escape', 'foggy 13h', 'x; touch /tmp/oops', ''])
def test_invalid_name_is_rejected_before_recording(tag, tmp_path):
    from foscar_camera_train.collect_label import main
    with pytest.raises(SystemExit) as error:
        main([tag, '--data-root', str(tmp_path / 'raw')])
    assert error.value.code == 2
    assert not (tmp_path / 'raw').exists()


def test_ambiguous_new_sessions_are_not_silently_combined(tmp_path):
    from foscar_camera_train.collect_label import new_session
    for hour in ('120000', '130000'):
        make_session(tmp_path / f'20260922_{hour}_foggy_13h', [10])
    with pytest.raises(RuntimeError, match='2개'):
        new_session(tmp_path, set(), 'foggy_13h')


def test_terminal_interrupt_reaches_real_launch_child_without_escalation(tmp_path):
    """Exercise Humble's interactive signal assumption without any ROS nodes/UDP."""
    pytest.importorskip('launch_ros')
    ready, stopped = tmp_path / 'ready', tmp_path / 'stopped'
    child_code = f'''
import pathlib, signal, time
def stop(signum, frame):
    pathlib.Path({str(stopped)!r}).touch()
    raise SystemExit(0)
signal.signal(signal.SIGINT, stop)
pathlib.Path({str(ready)!r}).touch()
while True:
    time.sleep(.02)
'''
    launch_file = tmp_path / 'signal_probe.launch.py'
    launch_file.write_text(
        'from launch import LaunchDescription\nfrom launch.actions import ExecuteProcess\n'
        'def generate_launch_description():\n'
        f'    return LaunchDescription([ExecuteProcess(cmd=[{sys.executable!r}, "-c", {child_code!r}])])\n')
    runner_code = (
        'import sys\nfrom foscar_camera_train.collect_label import run_child\n'
        'assert sys.stdin.isatty()\n'
        f'run_child(["ros2", "launch", "--noninteractive", {str(launch_file)!r}], collecting=True)\n')
    env = dict(os.environ)
    env['PYTHONPATH'] = str(Path(__file__).resolve().parents[1]) + os.pathsep + env.get('PYTHONPATH', '')
    master, slave = pty.openpty()
    process = subprocess.Popen([sys.executable, '-u', '-c', runner_code],
                               stdin=slave, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               env=env, text=True, start_new_session=True)
    os.close(slave)
    try:
        wait_for_file(ready, process)
        process.send_signal(signal.SIGINT)
        output, _ = process.communicate(timeout=10)
        assert process.returncode == 0, output
        assert stopped.exists(), output
        assert 'escalating' not in output
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGTERM)
            process.communicate(timeout=15)
        os.close(master)
