#!/usr/bin/env python3
"""Local launch checks; live UDP/ROS health still requires a running system."""
import importlib.util
import ipaddress
import math
from pathlib import Path
import socket
import sys

from control_core import valid_zones
from udp_protocol import validate_ports


def preflight(root, args, probe_ports=True, load_model=True):
    import numpy as np
    import yaml
    def path_arg(name, default): return Path(args.get(name, str(default))).expanduser()
    path = path_arg('path_file', root/'paths/global_path.txt')
    sensor = path_arg('sensor_config', root/'config/vehicle_sensor.yaml')
    missions = path_arg('mission_config', root/'config/missions.yaml')
    model = path_arg('pt_path', root/'weights/1027_40epoch.pt')
    onnx = path_arg('model_path', root/'weights/traffic.onnx')
    points = np.loadtxt(path, usecols=(0, 1))
    if points.ndim != 2 or len(points) < 3 or not np.isfinite(points).all(): raise ValueError('invalid path')
    length = float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
    cfg = yaml.safe_load(missions.read_text(encoding='utf-8'))
    device = yaml.safe_load(sensor.read_text(encoding='utf-8'))
    if not valid_zones(cfg.get('zones'), length) or len(cfg.get('intersections', [])) != 4:
        raise ValueError('invalid mission zones/intersections')
    from traffic_core import TrafficDecision
    if not TrafficDecision(cfg['intersections'], True).verified:
        raise ValueError('invalid traffic mission positions/ROI')
    xyz = device.get('lidar_xyz', [])
    if len(xyz) != 3 or not all(math.isfinite(float(x)) for x in xyz): raise ValueError('invalid LiDAR TF')
    defaults = {'gps_port': 1111, 'imu_port': 1112, 'status_port': 909,
                'camera_port': 9297, 'lidar_port': 2368, 'command_source_port': 9094}
    ports = {k: int(args.get(k, v)) for k, v in defaults.items()}
    validate_ports(ports)
    destination_port = int(args.get('command_port', 9093))
    if not 1 <= destination_port <= 65535: raise ValueError('invalid command destination port')
    if args.get('morai_ip') and ipaddress.ip_address(args['morai_ip']).version != 4:
        raise ValueError('morai_ip must be an IPv4 address')
    if probe_ports:
        sockets = []
        try:
            for name, port in ports.items():
                if name == 'lidar_port' and args.get('start_lidar_driver', 'true').lower() == 'false': continue
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sockets.append(sock)
                sock.bind((args.get('bind_ip', '0.0.0.0'), port))
        except OSError as exc:
            raise RuntimeError('UDP local port unavailable: %s=%s (%s)' % (name, port, exc))
        finally:
            for sock in sockets: sock.close()
    print('Path: %d points, %.2f m; mission entries=4; LiDAR TF=%s' % (len(points), length, xyz))
    print('UDP local ports:', ports, 'command destination:', args.get('morai_ip', '<missing>'), destination_port)
    print('Sensor reference verified:', bool(device.get('sensor_reference_verified', False)),
          'unverified mission sections:', len(cfg.get('unverified_sections', [])))
    if load_model:
        if model.is_file() and importlib.util.find_spec('ultralytics') and importlib.util.find_spec('torch'):
            from ultralytics import YOLO
            from traffic_core import LABELS
            actual = YOLO(str(model)).names
            if len(actual) != len(LABELS) or set(actual.values()) != set(LABELS):
                raise ValueError('model traffic classes do not match: %s' % actual)
            print('PT model loaded, classes:', actual)
        elif onnx.is_file() and importlib.util.find_spec('onnxruntime'):
            import onnxruntime as ort
            session = ort.InferenceSession(str(onnx), providers=['CPUExecutionProvider'])
            print('ONNX model loaded, input:', session.get_inputs()[0].shape)
        else:
            raise RuntimeError('no loadable PT or ONNX model; install corresponding dependencies')
    print('Live ROS topics, incoming packets, camera/LiDAR data and MORAI handover are not verified by preflight.')
    return True


def main():
    args = dict(value.split(':=', 1) for value in sys.argv[1:] if ':=' in value)
    required = ('rospy', 'numpy', 'scipy', 'pyproj', 'cv2', 'yaml', 'simul_msgs.msg', 'morai_msgs.msg')
    for name in required:
        try: __import__(name)
        except ImportError as exc: raise SystemExit('Missing dependency %s: %s' % (name, exc))
    if not args.get('morai_ip'): raise SystemExit('Pass morai_ip:=MORAI_IP')
    try: preflight(Path(__file__).resolve().parents[1], args)
    except Exception as exc: raise SystemExit('Preflight failed: %s' % exc)


if __name__ == '__main__': main()
