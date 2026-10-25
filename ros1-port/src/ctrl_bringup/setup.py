from setuptools import setup, find_packages
from catkin_pkg.python_setup import generate_distutils_setup

setup(**generate_distutils_setup(packages=find_packages(include=['ctrl_bringup', 'ctrl_bringup.*']), package_dir={'': '.'}))
