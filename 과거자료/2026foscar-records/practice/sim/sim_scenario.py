#!/usr/bin/env python3
"""Load a saved MORAI scenario over UDP (practice use): resets ego + NPC + objects.

Same wire format as MORAI-NetworkModule lib/define/ScenarioLoad.py (packed, 69 bytes).
The simulator's "Scenario Load" item listens on sim_ip:9095 here (control goes to 9093).
Without --send it only prints the packet. Loading replaces what is on screen, so save the
current layout in the simulator first.

    python3 practice/sim/sim_scenario.py M_TUNNEL                 # dry run
    python3 practice/sim/sim_scenario.py M_TUNNEL --send          # ego/npc/pedestrian/object reset
    python3 practice/sim/sim_scenario.py M_TUNNEL --send --network   # + saved network connections

Loading a scenario dropped the ego-network publishers (status 9099, ObjectInfo, collision)
on 2026-09-26 while sensors (GPS/IMU/lidar) continued. --network restores the connections
saved inside the scenario file (MORAI: load_network_connection_data).
"""
import argparse
import socket
import struct
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]


def packet(name, delete_all=False, network=False, ego=True, npc=True, pedestrian=True, obj=True, pause=False):
    raw = name.encode('ascii')
    if len(raw) > 30:
        raise ValueError('scenario name longer than 30 bytes')
    body = raw.ljust(30, b' ') + bytes([delete_all, network, ego, npc, pedestrian, obj, pause])
    return b'#ScenarioLoad$' + struct.pack('<i3i', len(body), 0, 0, 0) + body + b'\r\n'


def sim_ip():
    """The simulator address, from the one place it is configured (the driving config's sim_ip)."""
    cfg = yaml.safe_load((ROOT/'src/foscar_bringup/config/low_speed.yaml').read_text())
    return next(v['ros__parameters']['sim_ip'] for v in cfg.values() if 'sim_ip' in v.get('ros__parameters', {}))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('name', help='scenario file name saved in the simulator')
    ap.add_argument('--ip', default=sim_ip())
    ap.add_argument('--port', type=int, default=9095)
    ap.add_argument('--network', action='store_true',
                    help='also load the network connections saved in the scenario (reconnects channels)')
    ap.add_argument('--keep-npc', action='store_true', help='do not reload NPC vehicles')
    ap.add_argument('--keep-objects', action='store_true', help='do not reload static objects')
    ap.add_argument('--pause', action='store_true', help='leave the simulator paused after loading')
    ap.add_argument('--send', action='store_true', help='actually send (default: dry run)')
    a = ap.parse_args()
    data = packet(a.name, network=a.network, npc=not a.keep_npc, pedestrian=not a.keep_npc,
                  obj=not a.keep_objects, pause=a.pause)
    assert len(data) == 69
    print(f'{len(data)} bytes -> {a.ip}:{a.port}  {data!r}')
    if a.send:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.sendto(data, (a.ip, a.port))
        print('sent')


if __name__ == '__main__':
    main()
