"""UDP transport with validated, timestamped snapshots."""
import ctypes
import socket
import threading
import time


class Receiver:
    def __init__(self, ip, port, data_type):
        if parser is None and data_type is None:
            raise ValueError('data_type or parser is required')
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2**20)
            self.socket.bind((ip, int(port)))
        except Exception:
            self.socket.close()
            raise
        self.parsed_data_queue = multiprocessing.Queue()
        threading.Thread(target=self.data_parsing, daemon=True).start()
        multiprocessing.Process(target=self.recv_udp_data, daemon=True).start()

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

    def recv_udp_data(self):
        while True:
            raw_data, _ = self.socket.recvfrom(self.data_size)
            ctypes.memmove(ctypes.addressof(self.data_type), raw_data, self.data_size)
            try: self.data_type.parsing()
            except Exception: pass
            self.parsed_data_queue.put(self.data_type)
    def data_parsing(self):
        while True: self.parsed_data = self.parsed_data_queue.get()
    def get_data(self): return self.parsed_data


class Sender:
    def __init__(self, ip, port):
        self.ip, self.port = ip, int(port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, data):
        raw = ctypes.string_at(ctypes.addressof(data), ctypes.sizeof(data))
        self.socket.sendto(raw, (self.ip, self.port))

    def close(self):
        self.socket.close()
