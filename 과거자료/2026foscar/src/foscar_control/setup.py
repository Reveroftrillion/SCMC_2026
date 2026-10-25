from setuptools import setup
from glob import glob
from os.path import isfile
name = 'foscar_control'
setup(name=name, version='0.1.0', packages=[name], data_files=[('share/ament_index/resource_index/packages', ['resource/'+name]), ('share/'+name, ['package.xml'])] + [('share/'+name+'/'+d, [f for f in glob(d+'/*') if isfile(f)]) for d in ['config','launch','routes','rviz'] if glob(d+'/*')], install_requires=['setuptools'], zip_safe=True, maintainer='FOSCAR', maintainer_email='team@example.com', description='Low speed MORAI UDP bridge, localization and path following', license='Apache-2.0', entry_points={'console_scripts': ['sensor_bridge = foscar_control.bridge:main', 'localization = foscar_control.localization:main', 'path_follower = foscar_control.follower:main']})
