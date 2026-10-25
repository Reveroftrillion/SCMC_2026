"""A summary must detect a real collision and malformed raw evidence."""
import json
import struct

from summarize_tunnel_run import collision_report


def collision_bytes(object_type, object_id):
    packet = bytearray(181)
    packet[:15] = b'#CollisionData$'
    struct.pack_into('<i', packet, 15, 148)
    struct.pack_into('<hh6f', packet, 39, object_type, object_id, *([0.]*6))
    packet[-2:] = b'\r\n'
    return packet.hex()


def test_raw_decoder_keeps_collision_and_parse_failure(tmp_path):
    path = tmp_path/'wire.jsonl'
    rows = [dict(port=9092, time=1., raw_hex=collision_bytes(-1, 0)),
            dict(port=9092, time=2., raw_hex=collision_bytes(4, 16)),
            dict(port=9092, time=3., raw_hex='bad'),
            dict(port=909, time=4., raw_hex='bad')]
    original = ''.join(json.dumps(row)+'\n' for row in rows)
    path.write_text(original)
    report = collision_report(path)
    assert report['packets'] == 3 and report['collision_packets'] == 1
    assert report['collisions'][0]['slots'][0]['object_id'] == 16
    assert len(report['parse_errors']) == 1
    assert path.read_text() == original
