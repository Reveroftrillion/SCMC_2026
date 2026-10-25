#!/usr/bin/env python3
"""Register the CTRL-only roslaunch shortcut without changing ROS or catkin."""
import argparse
from datetime import datetime
from pathlib import Path
import shlex

BEGIN = '# >>> CTRL ROS1 launch >>>'
END = '# <<< CTRL ROS1 launch <<<'


def install(bashrc, setup):
    original = bashrc.read_bytes() if bashrc.exists() else b''
    text = original.decode('utf-8')
    if text.count(BEGIN) != text.count(END) or text.count(BEGIN) > 1:
        raise ValueError('Incomplete or duplicate CTRL block; refusing to edit bashrc')
    line = 'source ' + shlex.quote(str(setup))
    block = BEGIN + '\n' + line + '\n' + END
    if BEGIN in text:
        start, end = text.index(BEGIN), text.index(END) + len(END)
        if end < start:
            raise ValueError('Reversed CTRL block; refusing to edit bashrc')
        updated = text[:start] + block + text[end:]
    else:
        updated = text + ('' if not text or text.endswith('\n') else '\n') + '\n' + block + '\n'
    if updated.encode('utf-8') == original:
        return None
    backup = None
    if bashrc.exists():
        backup = bashrc.with_name(bashrc.name + '.ctrl-backup-' + datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
        with backup.open('xb') as stream:
            stream.write(original)
        backup.chmod(bashrc.stat().st_mode & 0o777)
    bashrc.write_bytes(updated.encode('utf-8'))
    return backup


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bashrc', type=Path, default=Path.home() / '.bashrc')
    args = parser.parse_args()
    setup = Path(__file__).resolve().parents[1] / 'docker/host_setup.bash'
    backup = install(args.bashrc.expanduser(), setup)
    print('CTRL shortcut registered in', args.bashrc)
    if backup:
        print('Previous configuration:', backup)
    print('Open a new terminal, or run: source', shlex.quote(str(setup)))


if __name__ == '__main__':
    main()
