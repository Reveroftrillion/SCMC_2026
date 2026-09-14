"""UDP transport with validated, timestamped snapshots."""
import ctypes
import logging
import socket
import threading
import time


class Receiver:
    def __init__(self, ip, port, data_type=None, parser=None, source_ip='', on_error=None):
        if parser is None and data_type is None:
            raise ValueError('data_type or parser is required')
        self.parser = parser or self._structure_parser(data_type)
        self.source_ip = source_ip
        self.on_error = on_error or logging.warning
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._data, self._received_at, self._sequence = None, None, 0
        self._last_warning = -float('inf')
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2**20)
            self.socket.bind((ip, int(port)))
            self.socket.settimeout(0.2)
        except Exception:
            self.socket.close()
            raise
        self._thread = threading.Thread(target=self._receive, daemon=True)
        self._thread.start()

    @staticmethod
    def _structure_parser(instance):
        cls, size = type(instance), ctypes.sizeof(instance)
        def parse(raw):
            if len(raw) != size:
                raise ValueError('packet length %d, expected %d' % (len(raw), size))
            value = cls.from_buffer_copy(raw)
            if hasattr(value, 'parsing'):
                raise ValueError('variable sensor data requires an explicit parser')
            return value
        return parse

    def _receive(self):
        while not self._stop.is_set():
            try:
                raw, peer = self.socket.recvfrom(65535)
                if self.source_ip and peer[0] != self.source_ip:
                    continue
                value = self.parser(raw)
                if value is not None:
                    with self._lock:
                        self._data = value
                        self._received_at = time.monotonic()
                        self._sequence += 1
            except socket.timeout:
                continue
            except Exception as exc:
                if self._stop.is_set():
                    break
                now = time.monotonic()
                if now - self._last_warning >= 2.0:
                    self._last_warning = now
                    self.on_error('UDP receive/parse: %s' % exc)

    def snapshot(self, max_age=0.5):
        with self._lock:
            if self._received_at is None or time.monotonic() - self._received_at > max_age:
                return None, self._sequence
            return self._data, self._sequence

    def get_data(self, max_age=0.5):
        return self.snapshot(max_age)[0]

    def close(self):
        self._stop.set()
        self.socket.close()
        if threading.current_thread() is not self._thread:
            self._thread.join(timeout=0.5)


class Sender:
    def __init__(self, ip, port, source_ip='', source_port=None):
        self.ip = ip
        self.port = int(port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        if source_port is not None:
            bind_ip = source_ip if source_ip else '0.0.0.0'
            self.socket.bind((bind_ip, int(source_port)))

    def send(self, data):
        raw = ctypes.string_at(ctypes.addressof(data), ctypes.sizeof(data))
        self.socket.sendto(raw, (self.ip, self.port))

    def close(self):
        self.socket.close()
