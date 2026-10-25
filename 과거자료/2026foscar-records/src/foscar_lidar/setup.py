from setuptools import setup
from glob import glob
from os.path import isfile
name = 'foscar_lidar'
setup(name=name, version='0.1.0', packages=[name], data_files=[('share/ament_index/resource_index/packages', ['resource/'+name]), ('share/'+name, ['package.xml'])] + [('share/'+name+'/'+d, [f for f in glob(d+'/*') if isfile(f)]) for d in ['launch','rviz'] if glob(d+'/*')], install_requires=['setuptools'], zip_safe=True, maintainer='FOSCAR', maintainer_email='team@example.com', description='VLP16 point cloud obstacle perception', license='Apache-2.0', entry_points={'console_scripts': ['obstacle_node = foscar_lidar.obstacle_node:main', 'lidar_bridge = foscar_lidar.lidar_bridge:main', 'lidar_monitor = foscar_lidar.lidar_monitor:main', 'lidar_view = foscar_lidar.lidar_view:main', 'fake_cloud = foscar_lidar.fake_cloud:main']})
