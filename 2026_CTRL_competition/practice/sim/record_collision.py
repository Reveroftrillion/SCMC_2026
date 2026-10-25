"""Record permitted MORAI CollisionData UDP packets, without sending control.

The packed schema comes from the supplied MORAI-NetworkModule. Preserve every
slot and raw bytes: empty slots / object type semantics are not guessed here.
"""
import argparse
import json
import socket
import struct
import time
from pathlib import Path


def decode_packet(raw):
    if len(raw)!=181 or raw[:15]!=b'#CollisionData$' or raw[-2:]!=b'\r\n':
        raise ValueError('invalid CollisionData framing')
    if struct.unpack_from('<i',raw,15)[0]!=148:
        raise ValueError('invalid CollisionData payload length')
    sec,nsec=struct.unpack_from('<ii',raw,31)
    slots=[]
    for i in range(5):
        kind,ident,*values=struct.unpack_from('<hh6f',raw,39+i*28)
        slots.append(dict(object_type=kind,object_id=ident,position=values[:3],global_offset=values[3:]))
    return dict(sim_sec=sec,sim_nsec=nsec,slots=slots)


class CollisionRecorder:
    def __init__(self,output,port=9092):
        self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        try:self.sock.bind(('0.0.0.0',port))
        except Exception:self.sock.close();raise
        self.sock.setblocking(False)
        self.file=Path(output).open('w');self.count=0

    def poll(self):
        for _ in range(128):
            try:raw,peer=self.sock.recvfrom(65535)
            except BlockingIOError:break
            row=dict(received_at=time.time(),source=peer,raw_hex=raw.hex())
            try:row.update(decode_packet(raw))
            except ValueError as exc:row['decode_error']=str(exc)
            self.file.write(json.dumps(row)+'\n');self.count+=1
        self.file.flush()

    def close(self):
        self.poll();self.sock.close();self.file.close()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True)
    parser.add_argument('--port',type=int,default=9092)
    parser.add_argument('--duration',type=float,default=10.)
    args=parser.parse_args();recorder=CollisionRecorder(args.output,args.port)
    try:
        end=time.monotonic()+args.duration
        while time.monotonic()<end:recorder.poll();time.sleep(.02)
    except KeyboardInterrupt:pass
    finally:recorder.close()
    print(f'Recorded {recorder.count} packets to {args.output}')
