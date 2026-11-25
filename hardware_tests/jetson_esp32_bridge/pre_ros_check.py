#!/usr/bin/env python3
"""Bounded bench tests. Requires lifted wheels and --enable-motion.
Channel names refer to firmware channels, not verified physical wheel sides.
"""
import argparse
import csv
import json
import pathlib
import time

import serial
from jetson_test import parse_frame


def run_suite(port, path, motion, results, watchdog_only=False, positive_only=False):
    """Keep one connection throughout; verify telemetry, not physical stop."""
    seq = 0
    buffer = bytearray()
    previous_ms = None
    latest = None
    last_ack = None
    last_progress = time.monotonic()
    last_feedback = last_progress
    next_send = 0
    # Set control lines before open; default asserted lines caused intermittent
    # silence on this board. Opening may still reset it, so handshake at zero.
    connection = serial.Serial(port=None, baudrate=115200, timeout=0,
                               write_timeout=0.1, exclusive=True)
    connection.dtr = False
    connection.rts = False
    connection.port = port
    with connection as s, path.open('w', newline='') as f:
        log = csv.writer(f)
        log.writerow(['host_s', 'phase', 'esp_ms', 'ack', 'ticks_l',
                      'ticks_r', 'pwm_l', 'pwm_r', 'timeout'])

        def send(left, right):
            nonlocal seq
            data = 'P,{},{},{}\n'.format(seq, left, right).encode('ascii')
            if s.write(data) != len(data):
                raise RuntimeError('Incomplete serial write')
            seq += 1

        def phase(name, seconds, command, startup=False):
            nonlocal buffer, previous_ms, latest, last_feedback
            nonlocal last_ack, last_progress, next_send
            start = time.monotonic()
            frames = []
            while time.monotonic() - start < seconds:
                now = time.monotonic()
                if command is not None and now >= next_send:
                    send(*command)
                    next_send = now + 0.05
                buffer.extend(s.read(min(s.in_waiting, 2048)))
                if len(buffer) > 8192:
                    raise RuntimeError('RX framing overflow')
                while b'\n' in buffer:
                    raw, _, tail = buffer.partition(b'\n')
                    buffer = bytearray(tail)
                    frame = parse_frame(raw)
                    if frame is None:
                        continue
                    if previous_ms is not None and frame[0] < previous_ms:
                        raise RuntimeError('ESP reset/wrap detected')
                    previous_ms = frame[0]
                    latest = frame
                    last_feedback = now
                    if frame[1] >= 0 and frame[1] != last_ack:
                        last_ack = frame[1]
                        last_progress = now
                    log.writerow([now, name] + frame)
                    frames.append((now, frame))
                    if not startup and command is not None:
                        if now-start > 0.15 and frame[6]:
                            raise RuntimeError('Unexpected command timeout')
                        # Allow up to one old feedback period after phase change.
                        if now-start > 0.15 and tuple(frame[4:6]) != command:
                            raise RuntimeError('Applied PWM differs from command')
                if not startup:
                    if now-last_feedback > 0.5:
                        raise RuntimeError('Feedback lost')
                    if command is not None and now-start > 0.15 and now-last_progress > 0.5:
                        raise RuntimeError('ACK stalled')
                time.sleep(0.002)
            return frames

        def record(name, frames):
            if not frames:
                raise RuntimeError('No frames in '+name)
            delta = [frames[-1][1][k]-frames[0][1][k] for k in (2, 3)]
            times = [t for t, _ in frames]
            item = {'name': name, 'status': 'pass', 'delta_ticks': delta,
                    'frames': len(frames), 'max_feedback_gap_s':
                    max([b-a for a, b in zip(times, times[1:])] or [0])}
            results['tests'].append(item)
            print('RESULT '+json.dumps(item), flush=True)
            return delta

        try:
            phase('connect_zero', 6, (0, 0), startup=True)
            now = time.monotonic()
            if (latest is None or now-last_feedback > 0.3 or
                    last_ack is None or now-last_progress > 0.3 or
                    latest[4:7] != [0, 0, 0]):
                raise RuntimeError('Zero-PWM handshake failed')
            record('baseline', phase('baseline', 3, (0, 0)))
            if not motion:
                record('stability_zero', phase('stability_zero', 30, (0, 0)))
                return
            cases = [('channel_left_positive', 80, 0),
                     ('channel_left_negative', -80, 0),
                     ('channel_right_positive', 0, 80),
                     ('channel_right_negative', 0, -80)]
            if watchdog_only:
                cases = []
            elif positive_only:
                cases = [case for case in cases if 'positive' in case[0]]
            for name, left, right in cases:
                print('TEST {}: PWM=({}, {}) 3s'.format(name, left, right), flush=True)
                frames = phase(name, 3, (left, right))
                confirmed = [frame for _, frame in frames
                             if frame[4:6] == [left, right]]
                if len(confirmed) < 40:
                    raise RuntimeError('Insufficient confirmed motion frames')
                delta = record(name, frames)
                # Include encoder coast-down in a separate stopped phase.
                record(name+'_stop', phase(name+'_stop', 2, (0, 0)))
                if not any(delta):
                    raise RuntimeError('No encoder movement: inspect wiring before continuing')
            if positive_only:
                return
            print('TEST watchdog: left PWM=80 for 1s, then silence', flush=True)
            phase('drive_left', 1, (80, 0))
            if latest[4:7] != [80, 0, 0]:
                raise RuntimeError('Motion not acknowledged before watchdog test')
            last_drive_ms = latest[0]
            start_silence = time.monotonic()
            frames = phase('silence_watchdog', 0.9, None)
            stops = [(t, frame) for t, frame in frames
                     if frame[4:7] == [0, 0, 1]]
            if not stops:
                raise RuntimeError('Watchdog did not report zero PWM')
            stop_t, stop_frame = stops[0]
            delay = stop_t-start_silence
            if delay > 0.6:
                raise RuntimeError('Watchdog zero-PWM report too late')
            if any(frame[4:6] != [0, 0] for t, frame in frames if t >= stop_t):
                raise RuntimeError('PWM resumed during command silence')
            print('WATCHDOG PASS: zero PWM reported {:.3f}s after silence'.format(delay), flush=True)
            results['watchdog'] = {
                'status': 'pass', 'host_observed_delay_s': delay,
                'esp_elapsed_since_last_drive_frame_ms': stop_frame[0]-last_drive_ms,
                'note': 'Telemetry confirms PWM=0, not physical wheel standstill.'}
            phase('recover_zero', 2, (0, 0))
            record('stability_zero', phase('stability_zero', 30, (0, 0)))
        finally:
            for _ in range(3):
                try:
                    send(0, 0)
                    time.sleep(0.03)
                except (serial.SerialException, OSError, RuntimeError):
                    break


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--enable-motion', action='store_true')
    parser.add_argument('--stationary-only', action='store_true')
    parser.add_argument('--watchdog-only', action='store_true')
    parser.add_argument('--positive-only', action='store_true',
                        help='Test each channel at +80 for 3s, with zero-PWM stops')
    args = parser.parse_args()
    if sum([args.stationary_only, args.watchdog_only, args.positive_only]) > 1:
        parser.error('Choose only one test mode')
    if not args.stationary_only and not args.enable_motion:
        parser.error('Lift wheels and pass --enable-motion')
    base = pathlib.Path(__file__).resolve().parent
    directory = base / 'logs' / ('pre-ros-' + time.strftime('%Y%m%d-%H%M%S'))
    directory.mkdir(parents=True)
    results = {'port': args.port, 'tests': [], 'physical_mapping': 'unconfirmed'}
    try:
        run_suite(args.port, directory/'telemetry.csv', not args.stationary_only,
                  results, args.watchdog_only, args.positive_only)
        results['zero_stability_status'] = 'not_run' if args.positive_only else 'pass'
        results['status'] = 'pass'
    except (Exception, KeyboardInterrupt) as error:
        results['status'] = 'failed'
        results['error'] = str(error)
        raise
    finally:
        (directory/'summary.json').write_text(json.dumps(results, indent=2)+'\n')
        print('REPORT '+str(directory/'summary.json'), flush=True)


if __name__ == '__main__':
    main()
