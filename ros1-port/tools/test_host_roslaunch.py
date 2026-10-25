"""Exercise host dispatch and shutdown with fake executables; no ROS or UDP."""
import importlib.util
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import tempfile
import time
import unittest

PORT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('installer', PORT / 'tools/install_host_roslaunch.py')
installer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(installer)


class HostLaunchTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.repo = self.root / "repo with space 'quote"
        self.docker = self.repo / 'ros1-port/docker'
        self.docker.mkdir(parents=True)
        for name in ('host_setup.bash', 'run.sh'):
            shutil.copy2(PORT / 'docker' / name, self.docker / name)
        self.bin = self.root / 'bin'
        self.bin.mkdir()
        for name in ('docker', 'roslaunch'):
            path = self.bin / name
            path.write_text('''#!/usr/bin/env python3
import json, os, signal, sys, time
from pathlib import Path
out=Path(os.environ['TEST_CALL'])
out.write_text(json.dumps(dict(command=Path(sys.argv[0]).name,args=sys.argv[1:])))
if os.environ.get('TEST_WAIT'):
    def stop(signum, frame):
        out.with_suffix('.signal').write_text(str(signum))
        sys.exit(0)
    signal.signal(signal.SIGINT, stop)
    out.with_suffix('.ready').touch()
    while True: time.sleep(.02)
sys.exit(int(os.environ.get('TEST_EXIT', '0')))
''')
            path.chmod(0o755)
        self.call = self.root / 'call.json'
        self.env = dict(os.environ, PATH=str(self.bin) + os.pathsep + os.environ['PATH'],
                        TEST_CALL=str(self.call))

    def command(self, *args):
        return ['bash', '-c', 'source "$1"; shift; roslaunch "$@"',
                'host-test', str(self.docker / 'host_setup.bash')] + list(args)

    def run_launch(self, *args):
        result = subprocess.run(self.command(*args), env=self.env, cwd=self.root,
                                capture_output=True, text=True, timeout=10)
        return result, json.loads(self.call.read_text())

    def test_ctrl_dispatches_from_any_directory_and_preserves_arguments(self):
        run_dir = '/workspace/ros1-port/docker/artifacts/run with spaces;literal'
        result, call = self.run_launch('ctrl_start.launch', 'drive:=false', 'bag:=false',
                                       'raw:=false', 'run_dir:=' + run_dir)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(call['command'], 'docker')
        self.assertEqual(call['args'][-6:], ['roslaunch', 'ctrl_start.launch', 'drive:=false',
                                            'bag:=false', 'raw:=false', 'run_dir:=' + run_dir])
        self.assertIn('NET_BIND_SERVICE', call['args'])
        self.assertIn('--network', call['args'])
        self.assertIn(str(self.repo) + ',dst=/workspace', ' '.join(call['args']))

    def test_default_launch_keeps_competition_defaults(self):
        _, call = self.run_launch('ctrl_start.launch')
        self.assertEqual(call['args'][-2:], ['roslaunch', 'ctrl_start.launch'])
        self.assertNotIn('drive:=false', call['args'])

    def test_other_roslaunch_commands_pass_through(self):
        for args in [('another_package', 'start.launch', 'arg:=with spaces'), ('--help',), ()]:
            with self.subTest(args=args):
                result, call = self.run_launch(*args)
                self.assertEqual(result.returncode, 0)
                self.assertEqual(call, dict(command='roslaunch', args=list(args)))

    def test_container_failure_is_reported(self):
        self.env['TEST_EXIT'] = '23'
        result, _ = self.run_launch('ctrl_start.launch')
        self.assertEqual(result.returncode, 23)

    def test_sigint_reaches_container_command(self):
        self.env['TEST_WAIT'] = '1'
        process = subprocess.Popen(self.command('ctrl_start.launch', 'drive:=false'),
                                   env=self.env, cwd=self.root, start_new_session=True,
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            deadline = time.monotonic() + 5
            while not self.call.with_suffix('.ready').exists() and time.monotonic() < deadline:
                self.assertIsNone(process.poll())
                time.sleep(.02)
            self.assertTrue(self.call.with_suffix('.ready').exists())
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=5)
            self.assertEqual(self.call.with_suffix('.signal').read_text(), str(int(signal.SIGINT)))
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()

    def test_install_preserves_configuration_backup_and_is_idempotent(self):
        bashrc = self.root / '.bashrc'
        original = b'# existing ROS workspace\nexport KEEP_ME=yes\n'
        bashrc.write_bytes(original)
        setup = self.docker / 'host_setup.bash'
        backup = installer.install(bashrc, setup)
        self.assertEqual(backup.read_bytes(), original)
        self.assertTrue(bashrc.read_bytes().startswith(original))
        self.assertIsNone(installer.install(bashrc, setup))
        result = subprocess.run(['bash', '-c', 'source "$1"; roslaunch ctrl_start.launch drive:=false',
                                 'install-test', str(bashrc)], env=self.env, timeout=10)
        self.assertEqual(result.returncode, 0)
        self.assertEqual(json.loads(self.call.read_text())['command'], 'docker')

    def test_malformed_install_block_is_left_untouched(self):
        for text in (installer.BEGIN, installer.END + '\n' + installer.BEGIN,
                     installer.BEGIN + installer.BEGIN + installer.END + installer.END):
            with self.subTest(text=text):
                bashrc = self.root / '.bashrc'
                bashrc.write_text(text)
                with self.assertRaises(ValueError):
                    installer.install(bashrc, self.docker / 'host_setup.bash')
                self.assertEqual(bashrc.read_text(), text)


if __name__ == '__main__':
    unittest.main()
