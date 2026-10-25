from setuptools import setup
from glob import glob
from os.path import isfile
name = 'foscar_camera_train'
setup(name=name, version='0.1.0', packages=[name], data_files=[('share/ament_index/resource_index/packages', ['resource/'+name]), ('share/'+name, ['package.xml'])] + [('share/'+name+'/'+d, [f for f in glob(d+'/*') if isfile(f)]) for d in ['launch','config'] if glob(d+'/*')], install_requires=['setuptools'], zip_safe=True, maintainer='FOSCAR', maintainer_email='team@example.com', description='Traffic light YOLO dataset collection and training (offline only, never launched during competition runs)', license='Apache-2.0', entry_points={'console_scripts': [
    # 수집기와 정차 노드. extract/roboflow_io/train/evaluate 는 평범한 스크립트라
    # `ml/bin/python -m foscar_camera_train.<모듈>` 로 직접 돌린다 (venv 문제를 피한다).
    'recorder_node = foscar_camera_train.recorder_node:main',
    'collection_stop_node = foscar_camera_train.dwell_node:main',
]})
