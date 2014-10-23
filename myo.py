from __future__ import print_function

from collections import defaultdict
import struct
import threading
import time

import serial
from serial.tools.list_ports import comports

def pack(fmt, *args):
    return struct.pack('<' + fmt, *args)
def unpack(fmt, *args):
    return struct.unpack('<' + fmt, *args)

class Packet(object):
    def __init__(self, ords):
        self.typ = ords[0]
        self.cls = ords[2]
        self.cmd = ords[3]
        self.payload = ''.join(map(chr, ords[4:]))

    def __repr__(self):
        return 'Packet(%02X, %02X, %02X, [%s])' % \
            (self.typ, self.cls, self.cmd,
             ' '.join('%02X' % ord(c) for c in self.payload))

class BT(object):
    '''Implements the non-Myo-specific details of the Bluetooth protocol.'''
    def __init__(self, tty):
        self.ser = serial.Serial(port=tty, baudrate=9600)
        self.buf = []
        self.lock = threading.Lock()
        self.handlers = []

    ## internal data-handling methods
    def recv_packet(self, timeout=None):
        t0 = time.time()
        self.ser.timeout = None
        while timeout is None or time.time() < t0 + timeout:
            if timeout is not None: self.ser.timeout = t0 + timeout - time.time()
            c = self.ser.read()
            if not c: return None

            ret = self.proc_byte(ord(c))
            if ret:
                if ret.typ == 0x80:
                    self.handle_event(ret)
                return ret

    def recv_packets(self, timeout=.5):
        res = []
        t0 = time.time()
        while time.time() < t0 + timeout:
            p = self.recv_packet(t0 + timeout - time.time())
            if not p: return res
            res.append(p)
        return res

    def proc_byte(self, c):
        if not self.buf:
            if c in [0x00, 0x80, 0x08, 0x88]:
                self.buf.append(c)
            return None
        elif len(self.buf) == 1:
            self.buf.append(c)
            self.packet_len = 4 + (self.buf[0] & 0x07) + self.buf[1]
            return None
        else:
            self.buf.append(c)

        if self.packet_len and len(self.buf) == self.packet_len:
            p = Packet(self.buf)
            self.buf = []
            return p
        return None

    def handle_event(self, p):
        for h in self.handlers:
            h(p)

    def add_handler(self, h):
        self.handlers.append(h)

    def remove_handler(self, h):
        try: self.handlers.remove(h)
        except ValueError: pass

    def wait_event(self, cls, cmd):
        res = [None]
        def h(p):
            if p.cls == cls and p.cmd == cmd:
                res[0] = p
        self.add_handler(h)
        while res[0] is None:
            self.recv_packet()
        self.remove_handler(h)
        return res[0]

    ## specific BLE commands
    def connect(self, addr):
        return self.send_command(6, 3, pack('6sBHHHH', ''.join(map(chr, addr)), 0, 6, 6, 64, 0))

    def get_connections(self):
        return self.send_command(0, 6)

    def discover(self):
        return self.send_command(6, 2, '\x01')

    def end_scan(self):
        return self.send_command(6, 4)

    def disconnect(self, h):
        return self.send_command(3, 0, pack('B', h))

    def read_attr(self, con, attr):
        self.send_command(4, 4, pack('BH', con, attr))
        return self.wait_event(4, 5)

    def write_attr(self, con, attr, val):
        self.send_command(4, 5, pack('BHB', con, attr, len(val)) + val)
        return self.wait_event(4, 1)

    def send_command(self, cls, cmd, payload='', wait_resp=True):
        s = pack('4B', 0, len(payload), cls, cmd) + payload
        self.ser.write(s)

        while True:
            p = self.recv_packet()

            ## no timeout, so p won't be None
            if p.typ == 0: return p

            ## not a response: must be an event
            self.handle_event(p)


class Myo(object):
    '''Implements the Myo-specific communication protocol.'''

    def __init__(self, tty=None):
        if tty is None:
            tty = self.detect_tty()
        if tty is None:
            raise ValueError('Myo dongle not found!')

        self.bt = BT(tty)
        self.emg_handlers = []
        self.imu_handlers = []

    def detect_tty(self):
        for p in comports():
            if 'PID=2458:0001' in p[2]:
                return p[0]

        return None

    def run(self):
        self.bt.recv_packet()

    def connect(self):
        ## stop everything from before
        self.bt.end_scan()
        self.bt.disconnect(0)
        self.bt.disconnect(1)
        self.bt.disconnect(2)

        ## start scanning
        print('scanning...')
        self.bt.discover()
        while True:
            p = self.bt.recv_packet()
            print('scan response:', p)
            if p.payload[15:] == '\x06\x42\x48\x12\x4A\x7F\x2C\x48\x47\xB9\xDE\x04\xA9\x01\x00\x06\xD5':
                addr = map(ord, p.payload[2:8])
                break
        self.bt.end_scan()

        ## connect and wait for status event
        conn_pkt = self.bt.connect(addr)
        self.conn = ord(conn_pkt.payload[-1])
        self.bt.wait_event(3, 0)

        ## get firmware version
        fw = self.bt.read_attr(self.conn, 0x17)
        _, _, _, _, v0, v1, v2, v3 = unpack('BHBBHHHH', fw.payload)
        print('firmware version: %d.%d.%d.%d' % (v0, v1, v2, v3))

        ## don't know what these do; Myo Connect sends them, though we get data
        ## fine without them
        self.bt.write_attr(self.conn, 0x19, '\x01\x02\x00\x00')
        self.bt.write_attr(self.conn, 0x2f, '\x01\x00')
        self.bt.write_attr(self.conn, 0x2c, '\x01\x00')
        self.bt.write_attr(self.conn, 0x32, '\x01\x00')
        self.bt.write_attr(self.conn, 0x35, '\x01\x00')

        ## enable EMG data
        self.bt.write_attr(self.conn, 0x28, '\x01\x00')
        ## enable IMU data
        self.bt.write_attr(self.conn, 0x1d, '\x01\x00')

        ## Sampling rate of the underlying EMG sensor, capped to 1000. If it's
        ## less than 1000, emg_hz is correct. If it is greater, the actual
        ## framerate starts dropping inversely. Also, if this is much less than
        ## 1000, EMG data becomes slower to respond to changes. In conclusion,
        ## 1000 is probably a good value.
        C = 1000
        emg_hz = 50
        ## strength of low-pass filtering of EMG data
        emg_smooth = 100

        imu_hz = 50

        ## send sensor parameters, or we don't get any data
        self.bt.write_attr(self.conn, 0x19, pack('BBBBHBBBBB', 2, 9, 2, 1, C, emg_smooth, C / emg_hz, imu_hz, 0, 0))

        ## add data handlers
        def handle_data(p):
            c, attr, typ = unpack('BHB', p.payload[:4])
            pay = p.payload[5:]

            if attr == 0x27:
                vals = unpack('8HB', pay)
                ## not entirely sure what the last byte is, but it's a bitmask that
                ## seems to indicate which sensors think they're being moved around or
                ## something
                emg = vals[:8]
                moving = vals[8]
                self.proc_emg(emg, moving)
            if attr == 0x1c:
                vals = unpack('10h', pay)
                quat = vals[:4]
                acc = vals[4:7]
                gyro = vals[7:10]
                self.proc_imu(quat, acc, gyro)

        self.bt.add_handler(handle_data)


    def disconnect(self):
        self.bt.disconnect(self.conn)

    def vibrate(self, length):
        if length in xrange(1, 4):
            ## first byte tells it to vibrate; purpose of second byte is unknown
            self.bt.write_attr(self.conn, 0x19, pack('3B', 3, 1, length))

    def add_emg_handler(self, h):
        self.emg_handlers.append(h)
    def add_imu_handler(self, h):
        self.imu_handlers.append(h)

    def proc_emg(self, emg, moving):
        for h in self.emg_handlers:
            h(emg, moving)
    def proc_imu(self, quat, acc, gyro):
        for h in self.imu_handlers:
            h(quat, acc, gyro)

last_vals = None
def plot(scr, vals):
    DRAW_LINES = False

    global last_vals
    if last_vals is None:
        last_vals = vals
        return

    D = 5
    scr.scroll(-D)
    scr.fill((0,0,0), (w - D, 0, w, h))
    for i, (u, v) in enumerate(zip(last_vals, vals)):
        if DRAW_LINES:
            pygame.draw.line(scr, (0,255,0),
                             (w - D, int(h/8 * (i+1 - u))),
                             (w, int(h/8 * (i+1 - v))))
            pygame.draw.line(scr, (255,255,255),
                             (w - D, int(h/8 * (i+1))),
                             (w, int(h/8 * (i+1))))
        else:
            c = int(255 * max(0, min(1, v)))
            scr.fill((c, c, c), (w - D, i * h / 8, D, (i + 1) * h / 8 - i * h / 8));

    pygame.display.flip()
    last_vals = vals

if __name__ == '__main__':
    import sys
    import pygame
    from pygame.locals import *

    w, h = 1200, 400
    scr = pygame.display.set_mode((w, h))

    m = Myo(sys.argv[1] if len(sys.argv) >= 2 else None)

    def proc_emg(emg, moving, times=[]):
        ## update pygame display
        plot(scr, [e / 2000. for e in emg])

        ## print framerate of received data
        times.append(time.time())
        if len(times) > 20:
            print((len(times) - 1) / (times[-1] - times[0]))
            times.pop(0)

    m.add_emg_handler(proc_emg)
    m.connect()

    try:
        while True:
            m.run()

            for ev in pygame.event.get():
                if ev.type == KEYDOWN:
                    if 49 <= ev.key < 52:
                        m.vibrate(ev.key - 48)
                elif ev.type == QUIT: raise Exception()

    except KeyboardInterrupt:
        pass
    finally:
        m.disconnect()
        print()
