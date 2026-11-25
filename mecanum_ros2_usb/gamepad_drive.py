#!/usr/bin/env python3
"""Read a DualShock-style Linux joystick and answer Jetson drive polls over SSH."""
import argparse
import array
import fcntl
import os
import struct
import subprocess

EVENT = struct.Struct('<IhBB')
AXIS_MAP = 0x80406A32
BUTTON_MAP = 0x81906A34


def mapped_indices(fd):
    axes = array.array('B', [0] * 64)
    buttons = array.array('H', [0] * 200)
    fcntl.ioctl(fd, AXIS_MAP, axes, True)
    fcntl.ioctl(fd, BUTTON_MAP, buttons, True)
    return [axes.index(code) for code in (1, 0, 3)], buttons


def stick(value):
    position = -max(-1.0, min(value / 32767.0, 1.0))
    if abs(position) <= 0.12:
        return 0.0
    return (position - (0.12 if position > 0 else -0.12)) / 0.88


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', default='/dev/input/js0')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--drive', action='store_true', help='enable motor output while holding L1')
    mode.add_argument('--follow', action='store_true', help='hold L1 to enable the Nav2 trial')
    args = parser.parse_args()
    with open(args.device, 'rb', buffering=0) as joystick:
        fd = joystick.fileno()
        os.set_blocking(fd, False)
        axis_indices, button_codes = mapped_indices(fd)
        l1 = button_codes.index(310)
        axes = {}
        pressed = None
        armed = False
        last = None
        command = [
            'ssh', '-i', os.path.expanduser('~/.ssh/jetson_codex_ed25519'),
            '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=6',
            'trong@192.168.1.112',
            'bash ~/mecanum_ros2_usb/run_gamepad_host.sh' +
            (' --follow' if args.follow else ' --drive' if args.drive else ''),
        ]
        if args.follow:
            print('Để cần analog ở giữa. Giữ L1 để xe bám đường, thả L1 để dừng.', flush=True)
        else:
            print('Cần trái: tiến/lùi và đi ngang. Cần phải ngang: xoay. Giữ L1 để lái.', flush=True)
            print('Chế độ lái.' if args.drive else 'Chế độ kiểm tra: motor khóa.', flush=True)
        checking = not (args.drive or args.follow)
        with subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                              text=True, bufsize=1) as connection:
            try:
                for line in connection.stdout:
                    if not line.startswith('POLL '):
                        print(line.rstrip(), flush=True)
                        continue
                    token = line.split()[1]
                    while True:
                        try:
                            event = os.read(fd, EVENT.size)
                        except BlockingIOError:
                            break
                        if len(event) != EVENT.size:
                            raise OSError('Mất kết nối tay cầm')
                        _, value, kind, index = EVENT.unpack(event)
                        if kind & 0x7f == 2:
                            axes[index] = value
                        elif kind & 0x7f == 1:
                            if index == l1:
                                pressed = bool(value)
                            if checking and not kind & 0x80:
                                print(f'Nút {index} (mã Linux {button_codes[index]}): '
                                      f'{"nhấn" if value else "thả"}', flush=True)
                    position = tuple(stick(axes.get(i, 0)) for i in axis_indices)
                    if pressed is False and position == (0.0, 0.0, 0.0):
                        armed = True
                    enabled = armed and pressed is True and (not args.follow or not any(position))
                    if checking:
                        state = tuple(round(v, 2) for v in position) + (pressed, armed, enabled)
                        if last != state:
                            print(f'Tay cầm: x={state[0]} y={state[1]} xoay={state[2]} '
                                  f'L1_vat_ly={state[3]} san_sang={state[4]} cho_phep_lai={state[5]}',
                                  flush=True)
                            last = state
                    values = position if enabled else (0.0, 0.0, 0.0)
                    connection.stdin.write(f'{token} {int(enabled)} ' + ' '.join(map(str, values)) + '\n')
            except KeyboardInterrupt:
                pass
            finally:
                connection.stdin.close()
            return connection.wait()


if __name__ == '__main__':
    raise SystemExit(main())
