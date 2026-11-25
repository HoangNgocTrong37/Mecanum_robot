#!/usr/bin/env python3
"""Timed, single-wheel PWM measurement; PWM is not a measured RPM."""
import argparse
import math
import signal
import time

import serial
from control_core import feedback


class Link:
    def __init__(self, port):
        self.port = port
        self.seq = 0
        self.pending = {}
        self.buf = bytearray()
        self.ready = False
        self.active = False
        self.last_ack = None
        self.last_status = None

    def send(self, values):
        payload = ('P,{},{},{},{},{}\n'.format(self.seq, *values)).encode('ascii')
        if self.port.write(payload) != len(payload):
            raise RuntimeError('Incomplete serial write')
        now = time.monotonic()
        self.pending = {s: v for s, v in self.pending.items() if now-v[0] < 0.5}
        self.pending[self.seq] = (now, list(values))
        self.seq = (self.seq + 1) % 1000000001

    def read(self):
        self.buf.extend(self.port.read(min(self.port.in_waiting, 1024)))
        if len(self.buf) > 4096:
            raise RuntimeError('Serial receive overflow')
        while b'\n' in self.buf:
            raw, _, tail = self.buf.partition(b'\n')
            self.buf[:] = tail
            line = raw.decode('ascii', errors='replace').strip()
            if line.startswith('ERROR,'):
                raise RuntimeError(line)
            if line == 'READY,MECANUM_PWM_V1':
                if self.active:
                    raise RuntimeError('ESP32 restarted; stopping')
                self.ready = True
            status = feedback(line)
            if status and status[1] in self.pending:
                sent_at, requested = self.pending.pop(status[1])
                if time.monotonic()-sent_at < 0.5 and status[6] == 0:
                    # Firmware may temporarily return 0 while guarding reversal.
                    if any(v not in (0, req) for v, req in zip(status[2:6], requested)):
                        raise RuntimeError('Unexpected applied PWM; stopping')
                    self.last_ack = time.monotonic()
                    self.last_status = status

    def handshake(self):
        deadline = time.monotonic() + 6
        while time.monotonic() < deadline:
            if self.ready:
                self.send([0]*4)
            else:
                self.port.write(b'?\n')
            time.sleep(0.05)
            self.read()
            if self.ready and self.last_ack is not None:
                self.active = True
                return
        raise RuntimeError('ESP32 identity / zero-command ACK failed')

    def run_for(self, seconds, values, report=False):
        deadline = time.monotonic() + seconds
        next_report = time.monotonic() + 1
        while time.monotonic() < deadline:
            self.read()
            if self.last_ack is None or time.monotonic()-self.last_ack > 0.5:
                raise RuntimeError('No fresh ESP32 ACK for 500 ms; stopping')
            self.send(values)
            if report and time.monotonic() >= next_report:
                status = self.last_status
                print('ESP32 PWM FL/FR/RL/RR={} timeout={}'.format(
                    status[2:6], status[6]), flush=True)
                next_report = time.monotonic() + 1
            time.sleep(0.05)

    def stop(self):
        for _ in range(3):
            self.send([0]*4)
            time.sleep(0.05)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--wheel', choices=['FL', 'FR', 'RL', 'RR'], default='FL')
    parser.add_argument('--pwm', type=int, default=60)
    parser.add_argument('--seconds', type=float, default=15)
    parser.add_argument('--check-only', action='store_true', help='Only send zero PWM')
    args = parser.parse_args()
    if not 0 <= args.pwm <= 100:
        parser.error('--pwm must be between 0 and 100')
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 20:
        parser.error('--seconds must be between 1 and 20')

    def interrupted(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    port = None
    link = None
    try:
        # Configure control lines before opening CP210x: avoid holding ESP32
        # in reset / bootloader through the board's EN and GPIO0 circuitry.
        port = serial.Serial(port=None, baudrate=115200, timeout=0,
                             write_timeout=0.05, exclusive=True)
        port.dtr = False
        port.rts = False
        port.port = args.port
        port.open()
        link = Link(port)
        link.handshake()
        print('ESP32 connected; zero PWM acknowledged.', flush=True)
        if args.check_only:
            link.run_for(1, [0]*4)
            print('CHECK_OK: only zero PWM sent.', flush=True)
            return 0
        print('Keep ALL wheels lifted. Ctrl+C stops. PWM is not a fixed RPM.', flush=True)
        for remaining in range(5, 0, -1):
            print('Starting in {}...'.format(remaining), flush=True)
            link.run_for(1, [0]*4)
        values = [0]*4
        values[['FL', 'FR', 'RL', 'RR'].index(args.wheel)] = args.pwm
        print('START {} PWM={} for {} seconds'.format(args.wheel, args.pwm, args.seconds),
              flush=True)
        link.run_for(args.seconds, values, report=True)
        return 0
    except KeyboardInterrupt:
        print('Interrupted; stopping.', flush=True)
        return 130
    except (OSError, serial.SerialException, RuntimeError) as error:
        print('ERROR: {}'.format(error), flush=True)
        return 1
    finally:
        if port is not None:
            try:
                if link is not None:
                    link.stop()
                    print('STOP: zero PWM sent; wheels may coast.', flush=True)
            except (OSError, serial.SerialException, RuntimeError) as error:
                print('Stop write failed: {}; ESP32 watchdog must stop motors.'.format(error),
                      flush=True)
            finally:
                port.close()


if __name__ == '__main__':
    raise SystemExit(main())
