"""MORAI UDP codecs; competition status layout supplied from prior observations."""
import ctypes
import math
import struct
import time
from collections import namedtuple


class Command(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [('header', ctypes.c_char * 14), ('length', ctypes.c_int32),
                ('aux', ctypes.c_int32 * 3), ('mode', ctypes.c_int8),
                ('gear', ctypes.c_int8), ('kind', ctypes.c_int8),
                ('velocity', ctypes.c_float), ('acceleration', ctypes.c_float),
                ('accel', ctypes.c_float), ('brake', ctypes.c_float),
                ('steer', ctypes.c_float), ('tail', ctypes.c_char * 2)]


def command_packet(accel, brake, steer, mode=2, gear=4):
    if mode not in (1, 2) or gear not in (1, 4):
        raise ValueError('unsupported control mode or gear')
    if not all(math.isfinite(v) for v in (accel, brake, steer)):
        accel, brake, steer = 0.0, 1.0, 0.0
    m = Command()
    m.header, m.length, m.mode, m.gear, m.kind, m.tail = b'#MoraiCtrlCmd$', 23, mode, gear, 1, b'\r\n'
    m.brake = max(0.0, min(1.0, brake))
    m.accel = 0.0 if m.brake > 0 else max(0.0, min(1.0, accel))
    m.steer = max(-1.0, min(1.0, steer))
    return bytes(m)


def safe_command(command, command_time, status_time, now, command_timeout=.3, status_timeout=.5):
    if (command is None or command_time is None or status_time is None or
            not 0 <= now-command_time <= command_timeout or
            not 0 <= now-status_time <= status_timeout):
        return (0.0, 1.0, 0.0)
    return command


class Status(ctypes.LittleEndianStructure):
    # EgoVehicleStatus candidate. This is NOT a verified Competition Status layout.
    _pack_ = 1
    _fields_ = [('header', ctypes.c_char*11), ('length', ctypes.c_int32),
                ('aux', ctypes.c_int32*3), ('sec', ctypes.c_int32), ('nsec', ctypes.c_int32),
                ('mode', ctypes.c_int8), ('gear', ctypes.c_int8), ('signed_vel', ctypes.c_float),
                ('map_id', ctypes.c_int32)] + [(n, ctypes.c_float) for n in (
                    'accel','brake','size_x','size_y','size_z','overhang','wheelbase','rear_overhang',
                    'pos_x','pos_y','pos_z','roll','pitch','yaw','vel_x','vel_y','vel_z',
                    'ang_x','ang_y','ang_z','accel_x','accel_y','accel_z','steer')] + [
                    ('link_id', ctypes.c_char*38), ('tires', ctypes.c_float*12), ('tail', ctypes.c_char*2)]


StatusValues = namedtuple('StatusValues', 'speed yaw steer gear mode')


def validate_ports(ports):
    if any(not isinstance(v, int) or not 1 <= v <= 65535 for v in ports.values()):
        raise ValueError('UDP ports must be integers in 1..65535')
    if len(set(ports.values())) != len(ports):
        raise ValueError('UDP local ports overlap: %s' % ports)


def status_diagnostic(raw):
    """Never infer a payload format from its length or log vehicle data."""
    return 'length=%d header=%r tail=%r' % (len(raw), raw[:11], raw[-2:])


def parse_status_packet(raw):
    """Return (SI status, source timestamp, format). Yaw is radians, speed m/s.

    181 layout is the user-supplied previously observed format. The 229 Ego
    definition comes from the local project and is diagnostic-only in bridge.
    """
    if len(raw) not in (181, 229) or raw[:11] != b'#MoraiInfo$' or raw[-2:] != b'\r\n':
        raise ValueError('unrecognized status packet framing')
    expected = 152 if len(raw) == 181 else 200
    if struct.unpack_from('<i', raw, 11)[0] != expected:
        raise ValueError('invalid status payload length')
    if len(raw) == 181:
        values = struct.unpack_from('<iibbfiff' + 'f' * 22 + '38s', raw, 27)
        if not all(math.isfinite(v) for v in values[:-1]):
            raise ValueError('nonfinite competition status')
        sec, nsec = values[:2]
        mode, gear = values[2:4]
        speed, yaw, steer = values[20], values[19], values[29]
        kind = 'competition'
    else:
        s = Status.from_buffer_copy(raw)
        sec, nsec, mode, gear = s.sec, s.nsec, s.mode, s.gear
        speed, yaw, steer = s.vel_x, s.yaw, s.steer
        kind = 'ego'
    if (sec < 0 or not 0 <= nsec < 1000000000
            or not all(math.isfinite(v) for v in (speed, yaw, steer))
            or abs(speed) > 250 or abs(yaw) > 3600 or abs(steer) > 720
            or mode not in (1, 2) or gear not in (0, 1, 2, 3, 4, 5)):
        raise ValueError('invalid status values or timestamp')
    return StatusValues(speed / 3.6, math.radians(yaw), steer, gear, mode), (sec, nsec), kind


def parse_status(raw):
    return parse_status_packet(raw)[0]


class StatusSequence:
    """Reject duplicate or backwards source timestamps; reset requires restart.

    Arrival staleness is separately checked using monotonic time by ControlGate.
    No comparison to wall-clock time is made because MORAI uses simulation time.
    """
    def __init__(self):
        self.last = None

    def accept(self, timestamp):
        if self.last is not None and timestamp <= self.last:
            raise ValueError('duplicate or out-of-order status timestamp')
        self.last = timestamp


class ControlGate:
    """Gate motion on a verified, fresh Auto/D report and perform handover.

    Output is (accel, brake, steer, mode, gear), or None to avoid seizing an
    initially manual vehicle. Loss of an already active state sends a brake.
    Handover is latched, and requires confirmed stationary Auto/D, then an
    observed P report before requesting Manual/P. These codes come from the
    local EgoCtrlCmd example, not a verified competition packet.
    """
    def __init__(self, status_timeout=.5, command_timeout=.3, stop_hold=1.):
        self.status_timeout, self.command_timeout, self.stop_hold = status_timeout, command_timeout, stop_hold
        self.last_status = None
        self.status_time = None
        self.active = False
        self.finished = False
        self.stop_since = None
        self.complete = False
        self.handover_request = None

    def observe(self, status, now):
        self.last_status, self.status_time = status, now
        if not self.finished and status.mode == 2 and status.gear == 4:
            self.active = True

    def invalidate(self):
        self.status_time = None

    def output(self, command, command_time, now, finished=False):
        self.finished = self.finished or (self.active and finished)
        if self.complete or not self.active:
            return None
        fresh = (self.status_time is not None and 0 <= now-self.status_time <= self.status_timeout)
        if not fresh:
            self.stop_since = None
            # Do not ask for a new gear or mode based on stale data.
            if self.handover_request == 'manual':
                return None
            if self.handover_request == 'park':
                return (0., 1., 0., 2, 1)
            if self.last_status and self.last_status.mode == 2 and self.last_status.gear in (1, 4):
                return (0., 1., 0., 2, self.last_status.gear)
            return None
        status = self.last_status
        if not self.finished:
            if status.mode != 2 or status.gear != 4:
                self.active = False
                return None
            return safe_command(command, command_time, self.status_time, now,
                                self.command_timeout, self.status_timeout) + (2, 4)
        # Never send a gear or mode transition while the vehicle is moving.
        if abs(status.speed) > .1:  # m/s; require near standstill before gear changes
            self.stop_since = None
            if status.mode == 2 and status.gear in (1, 4):
                return (0., 1., 0., 2, status.gear)
            return None
        if status.mode == 1 and status.gear == 1:
            self.complete = True
            return None
        if status.mode != 2 or status.gear not in (1, 4):
            return None
        if self.stop_since is None:
            self.stop_since = now
        if now-self.stop_since < self.stop_hold:
            return (0., 1., 0., 2, status.gear)
        if status.gear == 4:
            self.handover_request = 'park'
            return (0., 1., 0., 2, 1)
        self.handover_request = 'manual'
        return (0., 1., 0., 1, 1)


def parse_imu(raw, layout='timestamped_115'):
    offset = {'timestamped_115': 33, 'legacy_107': 25}[layout]
    if len(raw) != offset+82 or not raw.startswith(b'#IMUData$') or raw[-2:] != b'\r\n':
        raise ValueError('invalid IMU packet')
    a = struct.unpack_from('<10d', raw, offset)
    norm = math.sqrt(sum(v*v for v in a[:4]))
    if not all(math.isfinite(v) for v in a) or norm < 1e-6:
        raise ValueError('invalid IMU data')
    return tuple(v/norm for v in a[:4]) + a[4:]


def parse_gps(raw):
    result = None
    for sentence in raw.rstrip(b'\x00').decode('ascii').splitlines():
        sentence = sentence.strip()
        if not sentence: continue
        if not sentence.startswith('$') or '*' not in sentence: raise ValueError('invalid NMEA')
        body, checksum = sentence[1:].split('*')
        check = 0
        for ch in body.encode('ascii'): check ^= ch
        if len(checksum) != 2 or check != int(checksum, 16): raise ValueError('bad NMEA checksum')
        f = body.split(',')
        if f[0] not in ('GPGGA', 'GNGGA'): continue
        quality = int(f[6] or '0')
        if quality == 0: return (0., 0., 0., 0)
        if quality < 0 or quality > 8: raise ValueError('invalid GPS quality')
        def coord(value, direction, positive, maximum):
            n = float(value); degrees, minutes = divmod(n, 100)
            if n < 0 or minutes >= 60 or direction not in positive: raise ValueError('invalid coordinate')
            val = degrees + minutes / 60
            if val > maximum: raise ValueError('invalid coordinate')
            return -val if direction in ('S', 'W') else val
        lat, lon, alt = coord(f[2], f[3], 'NS', 90), coord(f[4], f[5], 'EW', 180), float(f[9])
        if not all(math.isfinite(v) for v in (lat, lon, alt)): raise ValueError('invalid GPS data')
        result = lat, lon, alt, quality
    return result


class CameraAssembler:
    def __init__(self): self.frame, self.data, self.index, self.started = None, bytearray(), 0, 0.
    def feed(self, raw):
        if len(raw) < 21 or raw[:3] != b'MOR': raise ValueError('invalid camera header')
        sec, nsec, index, size = struct.unpack_from('<4I', raw, 3)
        if size > 64979 or len(raw) not in (21+size, 65000) or raw[-2:] not in (b'AI', b'EI'):
            raise ValueError('invalid camera fragment')
        now, frame = time.monotonic(), (sec, nsec)
        if index == 0: self.frame, self.data, self.index, self.started = frame, bytearray(), 0, now
        if frame != self.frame or index != self.index or now-self.started > .5 or len(self.data)+size > 8*1024*1024:
            self.frame, self.data = None, bytearray()
            raise ValueError('missing/reordered camera fragment')
        self.data.extend(raw[19:19+size]); self.index += 1
        if raw[-2:] == b'EI':
            jpeg = bytes(self.data); self.frame, self.data = None, bytearray()
            if not jpeg.startswith(b'\xff\xd8') or not jpeg.endswith(b'\xff\xd9'): raise ValueError('bad JPEG')
            return jpeg
        return None
