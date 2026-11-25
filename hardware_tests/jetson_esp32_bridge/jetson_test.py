#!/usr/bin/env python3
"""USB serial test, Python >=3.6 + pyserial. No ROS dependency, no PID.
Default: stationary encoder test. Nonzero commands require --enable-motion.
"""
import argparse
import csv
import time
import serial


def parse_frame(raw):
    try:
        fields = raw.decode('ascii').strip().split(',')
        if len(fields) != 8 or fields[0] != 'E':
            return None
        values = [int(x) for x in fields[1:]]
        ms, seq, left, right, pl, pr, timeout = values
        if not (0 <= ms <= 0xffffffff and -1 <= seq <= 1000000000
                and -160 <= pl <= 160 and -160 <= pr <= 160
                and timeout in (0, 1)):
            return None
        return values
    except (UnicodeDecodeError, ValueError):
        return None


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', required=True)
    p.add_argument('--left', type=int, default=0, help='signed PWM, NOT RPM')
    p.add_argument('--right', type=int, default=0, help='signed PWM, NOT RPM')
    p.add_argument('--seconds', type=float, default=10)
    p.add_argument('--enable-motion', action='store_true')
    p.add_argument('--csv', default='encoder_log.csv')
    a = p.parse_args()
    if not (-160 <= a.left <= 160 and -160 <= a.right <= 160):
        p.error('PWM must be within -160..160')
    if not (0 < a.seconds <= 120):
        p.error('--seconds must be within (0, 120]')
    if (a.left or a.right) and not a.enable_motion:
        p.error('Lift wheels, then add --enable-motion to allow nonzero PWM')

    seq = 0
    def send(s, left, right):
        nonlocal seq
        payload = 'P,{},{},{}\n'.format(seq, left, right).encode('ascii')
        if s.write(payload) != len(payload):
            raise RuntimeError('Incomplete serial write')
        seq = (seq + 1) % 1000000001

    # Opening some USB-UART boards resets the ESP32. Start with zero commands.
    with serial.Serial(a.port, 115200, timeout=0, write_timeout=0.1,
                       exclusive=True) as s, open(a.csv, 'w', newline='') as f:
        log = csv.writer(f)
        log.writerow(['host_monotonic_s', 'esp_ms', 'ack_seq', 'left_ticks',
                      'right_ticks', 'left_applied_pwm', 'right_applied_pwm',
                      'timed_out'])
        buffer = bytearray()
        start = time.monotonic()
        next_send = start
        last_feedback = None
        last_ack = None
        last_ack_progress = start
        prev_ms = None
        motion_start = None
        last_print = start - 1
        try:
            print('Connecting with zero PWM; waiting for feedback...')
            while True:
                now = time.monotonic()
                if now >= next_send:
                    moving = motion_start is not None
                    send(s, a.left if moving else 0, a.right if moving else 0)
                    next_send = now + 0.05  # 20 Hz heartbeat
                data = s.read(min(s.in_waiting, 2048))
                if data:
                    buffer.extend(data)
                    if len(buffer) > 8192:
                        raise RuntimeError('RX framing overflow')
                    while b'\n' in buffer:
                        raw, _, tail = buffer.partition(b'\n')
                        buffer = bytearray(tail)
                        frame = parse_frame(raw)
                        if frame is None:
                            continue  # Ignore startup messages/malformed frames.
                        ms, ack, tl, tr, pl, pr, timeout = frame
                        if prev_ms is not None and ms < prev_ms:
                            raise RuntimeError('ESP clock reset/wrap: stopping test')
                        prev_ms = ms
                        last_feedback = now
                        if ack >= 0 and ack != last_ack:
                            last_ack = ack
                            last_ack_progress = now
                        log.writerow([now] + frame)
                        if now - last_print >= 0.2:
                            print('ticks L={} R={} | PWM L={} R={} | timeout={}'
                                  .format(tl, tr, pl, pr, timeout))
                            last_print = now
                        if motion_start is not None and timeout:
                            raise RuntimeError('ESP command watchdog triggered')
                if motion_start is None:
                    if (now-start >= 2 and last_feedback is not None
                            and now-last_feedback < 0.3 and last_ack is not None
                            and now-last_ack_progress < 0.3):
                        motion_start = now
                        print('Test started: PWM L={} R={}, duration={} s'
                              .format(a.left, a.right, a.seconds))
                    elif now-start > 6:
                        raise RuntimeError('No working bidirectional link')
                else:
                    if last_feedback is None or now-last_feedback > 0.5:
                        raise RuntimeError('Encoder feedback lost')
                    if now-last_ack_progress > 0.5:
                        raise RuntimeError('Commands no longer acknowledged')
                    if now-motion_start >= a.seconds:
                        break
                time.sleep(0.002)
        finally:
            # Best effort stop; firmware independently stops after 300 ms silence.
            for _ in range(3):
                try:
                    send(s, 0, 0)
                    time.sleep(0.03)
                except (serial.SerialException, OSError, RuntimeError):
                    break
            print('Stop requested. CSV: {}'.format(a.csv))


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Interrupted')
    except (serial.SerialException, OSError, RuntimeError) as error:
        raise SystemExit('ERROR: {}'.format(error))
