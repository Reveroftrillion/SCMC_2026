"""ROS-independent MORAI packet parsers."""
import ctypes
import math
import struct
import time


def finite(values):
    if not all(math.isfinite(x) for x in values):
        raise ValueError('non-finite packet value')


def status_parser(cls, header):
    size = ctypes.sizeof(cls)
    def parse(raw):
        if len(raw) != size or not raw.startswith(header) or raw[-2:] != b'\r\n':
            raise ValueError('status length=%d expected=%d header=%r' % (len(raw), size, raw[:11]))
        value = cls.from_buffer_copy(raw)
        finite((value.vel_x, value.yaw))
        return value
    return parse


def _coordinate(value, direction, latitude):
    number = float(value)
    degrees, minutes = divmod(number, 100)
    allowed = ('N', 'S') if latitude else ('E', 'W')
    if direction not in allowed or number < 0 or minutes >= 60:
        raise ValueError('invalid NMEA coordinate')
    result = degrees + minutes / 60.0
    if not math.isfinite(result) or result > (90 if latitude else 180):
        raise ValueError('NMEA coordinate out of range')
    return -result if direction in ('S', 'W') else result


class GPSParser:
    """GGA provides position and altitude together; RMC does not overwrite it."""
    def __call__(self, raw):
        result = None
        for sentence in raw.rstrip(b'\x00').decode('ascii').splitlines():
            sentence = sentence.strip()
            if not sentence:
                continue
            body, checksum = sentence[1:].split('*')
            check = 0
            for char in body.encode('ascii'):
                check ^= char
            if not sentence.startswith('$') or len(checksum) != 2 or check != int(checksum, 16):
                raise ValueError('NMEA checksum mismatch')
            fields = body.split(',')
            if fields[0] not in ('GPGGA', 'GNGGA'):
                continue
            quality = int(fields[6] or '0')
            if quality < 0 or quality > 8:
                raise ValueError('invalid NMEA fix quality')
            if quality == 0:
                result = (0.0, 0.0, 0.0, 0)
                continue
            lat = _coordinate(fields[2], fields[3], True)
            lon = _coordinate(fields[4], fields[5], False)
            altitude = float(fields[9])
            finite((altitude,))
            result = (lat, lon, altitude, quality)
        return result


def imu_parser(layout):
    if layout not in ('timestamped_115', 'legacy_107'):
        raise ValueError('imu_layout must be timestamped_115 or legacy_107')
    offset = 33 if layout == 'timestamped_115' else 25
    def parse(raw):
        if len(raw) != offset + 82 or not raw.startswith(b'#IMUData$') or raw[-2:] != b'\r\n':
            raise ValueError('IMU length=%d configured=%s' % (len(raw), layout))
        values = struct.unpack_from('<10d', raw, offset)
        finite(values)
        norm = math.sqrt(sum(x*x for x in values[:4]))
        if not math.isfinite(norm) or norm < 1e-6:
            raise ValueError('zero IMU quaternion')
        return tuple(x / norm for x in values[:4]) + values[4:]
    return parse


class CameraParser:
    def __init__(self, timeout=0.5, max_bytes=8*1024*1024):
        self.timeout, self.max_bytes = timeout, max_bytes
        self.frame, self.buffer, self.next_index, self.started = None, bytearray(), 0, 0.0

    def __call__(self, raw):
        if len(raw) < 21 or raw[:3] != b'MOR':
            raise ValueError('invalid camera header/length (GT packets not accepted)')
        sec, nsec, index, size = struct.unpack_from('<4I', raw, 3)
        if size > 64979 or len(raw) not in (21 + size, 65000) or raw[-2:] not in (b'AI', b'EI'):
            raise ValueError('invalid camera block size/tail')
        now, frame = time.monotonic(), (sec, nsec)
        if index == 0:
            self.buffer = bytearray()
            self.frame, self.next_index, self.started = frame, 0, now
        if frame != self.frame or index != self.next_index or now - self.started > self.timeout:
            self.frame = None
            self.buffer.clear()
            raise ValueError('camera frame incomplete, reordered, or expired')
        if len(self.buffer) + size > self.max_bytes:
            self.frame = None
            self.buffer.clear()
            raise ValueError('camera frame exceeds memory limit')
        self.buffer.extend(raw[19:19 + size])
        self.next_index += 1
        if raw[-2:] == b'EI':
            jpeg = bytes(self.buffer)
            self.frame = None
            self.buffer.clear()
            if not jpeg.startswith(b'\xff\xd8') or not jpeg.endswith(b'\xff\xd9'):
                raise ValueError('incomplete JPEG')
            return jpeg
        return None


def command_values(accel, brake, steering):
    finite((accel, brake, steering))
    brake = min(1.0, max(0.0, brake))
    return (0.0 if brake > 0 else min(1.0, max(0.0, accel)),
            brake, min(1.0, max(-1.0, steering)))

def watchdog_values(values, command_time, status_time, now, command_timeout, status_timeout):
    if (values is None or command_time is None or status_time is None
            or now - command_time > command_timeout or now - status_time > status_timeout):
        return (0.0, 1.0, 0.0)
    return values
