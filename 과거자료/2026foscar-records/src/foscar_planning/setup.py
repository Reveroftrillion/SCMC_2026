from setuptools import setup
from glob import glob
from os.path import isfile
name = 'foscar_planning'
setup(name=name, version='0.1.0', packages=[name, name+'.states'], data_files=[('share/ament_index/resource_index/packages', ['resource/'+name]), ('share/'+name, ['package.xml'])] + [('share/'+name+'/'+d, [f for f in glob(d+'/*') if isfile(f)]) for d in ['config', 'launch', 'rviz'] if glob(d+'/*')], install_requires=['setuptools'], zip_safe=True, maintainer='FOSCAR', maintainer_email='team@example.com', description='Behavior FSM: events -> DrivingTarget', license='Apache-2.0', entry_points={'console_scripts': ['behavior_node = foscar_planning.behavior_node:main', 'bev_node = foscar_planning.bev_node:main', 'race_hud_node = foscar_planning.race_hud_node:main']})
