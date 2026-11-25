"""Pure logic, no ROS dependency. Body frame x forward, y left, yaw CCW.
Requires standard 45-degree mecanum configuration matching the documented matrix.
"""
import math


def to_pwm(vx, vy, wz, radius, lx, ly, gain, limit):
    values = (vx, vy, wz, radius, lx, ly, gain, limit)
    if not all(math.isfinite(x) for x in values):
        raise ValueError('Non-finite input')
    if min(radius, lx, ly, gain) <= 0 or not 1 <= limit <= 160:
        raise ValueError('Invalid geometry, gain or PWM limit')
    a = lx + ly
    omega = [(vx-vy-a*wz)/radius, (vx+vy+a*wz)/radius,
             (vx+vy-a*wz)/radius, (vx-vy+a*wz)/radius]
    pwm = [gain*w for w in omega]
    # Common scaling preserves requested wheel ratios; no independent clipping.
    scale = max(1.0, max(abs(x) for x in pwm)/limit)
    return [int(round(x/scale)) for x in pwm]


def select_command(command, received_at, now, timeout, enabled):
    if not enabled or received_at is None or now-received_at >= timeout:
        return (0.0, 0.0, 0.0)
    return command


def feedback(line):
    f = line.split(',')
    if len(f) != 8 or f[0] != 'T':
        return None
    try:
        v = [int(x) for x in f[1:]]
    except ValueError:
        return None
    if not (0 <= v[0] <= 0xffffffff and -1 <= v[1] <= 1000000000
            and all(abs(x) <= 160 for x in v[2:6]) and v[6] in (0, 1)):
        return None
    return v
