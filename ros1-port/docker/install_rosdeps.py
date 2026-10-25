#!/usr/bin/env python3
"""Resolve ROS native dependencies with rosdep, install apt packages in one batch.

Skipped Python keys are already supplied by the checked Python lock. This avoids
ROS apt-repository-specific '*-modules' package names unavailable on Jammy.
"""
from pathlib import Path
import shlex
import subprocess

provided = ('python3-catkin-pkg', 'python3-catkin-pkg-modules', 'python3-rospkg',
            'python3-rospkg-modules', 'python3-rosdep', 'python3-rosdep-modules',
            'python3-rosdistro', 'python3-rosdistro-modules', 'python3-empy', 'python3-numpy',
            'python3-vcstools', 'python3-wstool', 'python3-rosinstall-generator')
result = subprocess.run(['rosdep', 'install', '--from-paths', 'src', '--ignore-src',
                         '--rosdistro', 'noetic', '--os=ubuntu:jammy',
                         '--skip-keys=' + ' '.join(provided), '--simulate', '-y'],
                        check=True, text=True, capture_output=True)
print(result.stdout, flush=True)
print(result.stderr, flush=True)
# Debian's PyNaCl pulls in cffi-backend, while pip metadata also requires cffi.
packages = {'python3-cffi'}
for line in result.stdout.splitlines():
    command = shlex.split(line)
    if command[:2] == ['apt-get', 'install']:
        packages.update(arg for arg in command[2:] if not arg.startswith('-'))
    elif command and command[0] in ('pip', 'pip3', 'sudo'):
        raise RuntimeError('Unexpected non-apt dependency command: ' + line)
Path('/opt/rmt/apt-dependencies.txt').write_text('\n'.join(sorted(packages)) + '\n')
if packages:
    subprocess.run(['apt-get', 'install', '-y', '--no-install-recommends', *sorted(packages)], check=True)
subprocess.run(['python3', '-m', 'pip', 'check'], check=True)
