from setuptools import setup, find_packages
from catkin_pkg.python_setup import generate_distutils_setup

setup(**generate_distutils_setup(packages=find_packages(include=['ctrl_lidar', 'ctrl_lidar.*']), package_dir={'': '.'}))
