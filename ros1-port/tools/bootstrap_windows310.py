#!/usr/bin/env python3
"""Prepare a workspace-only Python 3.10 interpreter for Windows verification.

No registry, global Python, PATH, or original repository files are changed.
This does not install ROS or establish Linux/Noetic compatibility.
"""
import hashlib
from pathlib import Path
import urllib.request
import zipfile


WS = Path(__file__).resolve().parents[1]
URL = 'https://www.python.org/ftp/python/3.10.11/python-3.10.11-embed-amd64.zip'
# Checksum published on the official Python 3.10.11 release page.
EXPECTED_MD5 = 'f1c0538b060e03cbb697ab3581cb73bc'


def main():
    downloads = WS / '.downloads'
    destination = WS / '.python310'
    downloads.mkdir(exist_ok=True)
    archive = downloads / 'python-3.10.11-embed-amd64.zip'
    if not archive.exists():
        urllib.request.urlretrieve(URL, archive)
    data = archive.read_bytes()
    if hashlib.md5(data).hexdigest() != EXPECTED_MD5:
        raise ValueError('Official Python archive checksum mismatch: ' + str(archive))
    destination.mkdir(exist_ok=True)
    with zipfile.ZipFile(archive) as files:
        files.extractall(destination)
    (destination / 'Lib/site-packages').mkdir(parents=True, exist_ok=True)
    (destination / 'python310._pth').write_text('python310.zip\n.\nLib/site-packages\nimport site\n', encoding='ascii')
    print(destination / 'python.exe')
    print('archive SHA256:', hashlib.sha256(data).hexdigest())


if __name__ == '__main__':
    main()
