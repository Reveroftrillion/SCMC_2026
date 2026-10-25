#!/usr/bin/env python3
"""Thin terminal output; preserve every input line in an exclusive raw log."""
import argparse
import re
import signal
import sys
import time


class ConsoleView:
    def __init__(self):
        self.seen = {}
        self.summary = False
        self.state = None

    def show(self, line, now):
        line = re.sub(r'\x1b\[[0-9;]*m', '', line)
        if any(word in line for word in ('[ERROR]', '[FATAL]', 'Traceback',
                                         'shutdown handoff', 'process has died')):
            return True
        if line.strip() == 'SUMMARY':
            self.summary = True
            return False
        if self.summary:
            if line.startswith(('process[', '[INFO]', '[WARN]', '[ERROR]')):
                self.summary = False
            else:
                return False
        if not line.strip():
            return False
        body = re.sub(r'^\[(?:INFO|WARN)\]\s*\[[^]]+\]:\s*', '', line)
        match = re.match(r'([A-Z_]+) \|', body)
        changed = match is not None and match[1] != self.state
        if match:
            self.state = match[1]
        if body.startswith('RX='):
            key = 'RX'
        elif match:
            key = 'follower:' + match[1]
        elif body.startswith(('SLALOM', 'BEV ', 'TUNNEL BOX', 'RB_')):
            key = ' '.join(body.split()[:2])
        else:
            key = re.sub(r'[-+]?\d+(?:\.\d+)?', '#', body).strip()
        period = 10. if '[WARN]' in line else 5.
        if changed or now-self.seen.get(key, -float('inf')) >= period:
            self.seen[key] = now
            return True
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log', required=True)
    args = parser.parse_args()
    # The launch on the same terminal receives Ctrl+C; keep draining its handoff.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    view = ConsoleView()
    with open(args.log, 'x', buffering=1) as raw:
        for line in sys.stdin:
            raw.write(line)
            if view.show(line, time.monotonic()):
                print(line, end='', flush=True)


if __name__ == '__main__':
    main()
