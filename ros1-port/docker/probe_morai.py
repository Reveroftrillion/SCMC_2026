#!/usr/bin/env python3
"""Receive only: inspect actual MORAI packets with the unchanged RMT parsers.

Do not run alongside the ROS bridges: this probe owns the destination ports.
Never binds a simulator source/control port and never sends any datagrams.
"""
import argparse
from collections import Counter
import importlib.util
import json
from pathlib import Path
import select
import socket
import struct
import time

import numpy as np

PORT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('rmt_wire', PORT / 'src/ctrl_control/ctrl_control/ctrl_core.py')
wire = importlib.util.module_from_spec(spec)
spec.loader.exec_module(wire)
CHANNELS = {909: 'competition_status', 9111: 'ego_status', 1111: 'gps',
            1112: 'imu', 2368: 'lidar', 9291: 'front_camera', 9293: 'side_camera',
            9295: 'opposite_camera', 9297: 'additional_camera'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--save-images', type=Path, help='save one decoded JPEG sample per camera')
    args = parser.parse_args()
    records = {port: dict(channel=name, packets=0, valid=0, errors=0, lengths=Counter(),
                          sources=Counter()) for port, name in CHANNELS.items()}
    sockets, buffers = {}, {}
    try:
        for port in CHANNELS:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                sock.bind(('127.0.0.1', port))
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
                sockets[sock] = port
            except OSError as exc:
                records[port]['bind_error'] = str(exc)
                sock.close()
        print('Receive-only probe listening: ' + ', '.join(map(str, sockets.values())), flush=True)
        started = time.monotonic()
        while sockets and time.monotonic() - started < args.seconds:
            for sock in select.select(list(sockets), [], [], .2)[0]:
                data, source = sock.recvfrom(65535)
                port = sockets[sock]
                record = records[port]
                record['packets'] += 1
                record['lengths'][len(data)] += 1
                record['sources'][str(source)] += 1
                record.setdefault('first_header_hex', data[:32].hex())
                try:
                    if source[0] != '127.0.0.1':
                        raise ValueError('unexpected source address')
                    if port in (909, 9111):
                        record['last_parsed'] = wire.parse_status(data)
                    elif port == 1111:
                        fixes = wire.parse_gps(data)
                        if not fixes:
                            raise ValueError('no GGA/RMC fixes')
                        record['last_parsed'] = fixes
                    elif port == 1112:
                        q, angular, linear, stamp = wire.parse_imu(data)
                        record['last_parsed'] = dict(quaternion=q.tolist(), angular=list(angular),
                                                     linear=list(linear), stamp=list(stamp))
                    elif port == 2368:
                        if len(data) != 1206 or any(data[k:k+2] != b'\xff\xee' for k in range(0, 1200, 100)):
                            raise ValueError('not VLP16 1206-byte packet with 12 block markers')
                        record['return_mode'] = data[-2]
                        record['sensor_model'] = data[-1]
                    else:
                        if len(data) < 21 or data[:3] != b'MOR':
                            raise ValueError('not MOR camera fragment')
                        sec, nsec, index, size = struct.unpack_from('<iiii', data, 3)
                        record['last_fragment'] = dict(sec=sec, nsec=nsec, index=index, size=size)
                        body = data[19:-2]
                        if 0 < size < len(body):
                            body = body[:size]
                        if body[:2] == b'\xff\xd8':
                            buffers[port] = bytearray()
                        buf = buffers.setdefault(port, bytearray())
                        buf.extend(body)
                        if data[-2:] == b'EI':
                            if buf[:2] != b'\xff\xd8' or buf[-2:] != b'\xff\xd9':
                                raise ValueError('incomplete JPEG')
                            import cv2
                            frame = cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_COLOR)
                            if frame is None:
                                raise ValueError('JPEG decode failed')
                            record['image_shape'] = list(frame.shape)
                            record['color_channels_differ'] = bool(np.any(frame[:, :, 0] != frame[:, :, 1]))
                            if args.save_images and 'sample_image' not in record:
                                args.save_images.mkdir(parents=True, exist_ok=True)
                                sample = args.save_images / ('camera-port-' + str(port) + '.jpg')
                                sample.write_bytes(buf)
                                record['sample_image'] = str(sample)
                            record['complete_frames'] = record.get('complete_frames', 0) + 1
                            buffers[port] = bytearray()
                        elif len(buf) > 4 * 1024 * 1024:
                            buffers[port] = bytearray()
                            raise ValueError('image exceeds original 4MiB guard')
                    record['valid'] += 1
                except Exception as exc:
                    record['errors'] += 1
                    record['last_error'] = str(exc)
                    record['last_error_header_hex'] = data[:32].hex()
    finally:
        for sock in sockets:
            sock.close()
    report = dict(scope='actual receive-only UDP; no ROS or control transmission',
                  elapsed_seconds=time.monotonic() - started, channels=records)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
