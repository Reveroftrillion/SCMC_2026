from setuptools import setup
from glob import glob
from os.path import isfile
name = 'foscar_camera'
setup(name=name, version='0.1.0', packages=[name], data_files=[('share/ament_index/resource_index/packages', ['resource/'+name]), ('share/'+name, ['package.xml'])] + [('share/'+name+'/'+d, [f for f in glob(d+'/*') if isfile(f)]) for d in ['launch','config','models'] if glob(d+'/*')], install_requires=['setuptools'], zip_safe=True, maintainer='FOSCAR', maintainer_email='team@example.com', description='MORAI camera UDP bridge and traffic light recognition', license='Apache-2.0', entry_points={'console_scripts': ['yolo_signal_node = foscar_camera.yolo_signal_node:main', 'camera_bridge = foscar_camera.camera_bridge:main', 'camera_view = foscar_camera.camera_view:main', 'gps_logger = foscar_camera.gps_logger:main']})
