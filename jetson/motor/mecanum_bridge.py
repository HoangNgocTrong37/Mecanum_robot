#!/usr/bin/env python3
"""ROS2 Twist -> open-loop mecanum PWM -> USB ESP32. Python 3.6+."""
import math
import time
import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from control_core import to_pwm, select_command, feedback


class MecanumBridge(Node):
    def __init__(self):
        super().__init__('mecanum_bridge')
        defaults = {
            'port': '/dev/ttyUSB0', 'enable_motion': False,
            'wheel_radius': 0.0, 'lx': 0.0, 'ly': 0.0,
            'pwm_per_rad_s': 0.0, 'pwm_limit': 100,
            'cmd_timeout': 0.3, 'max_vx': 0.2, 'max_vy': 0.2, 'max_wz': 0.5,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        for key in ('cmd_timeout', 'max_vx', 'max_vy', 'max_wz'):
            if not math.isfinite(self.p[key]) or self.p[key] <= 0:
                raise ValueError('Invalid parameter: '+key)
        if self.p['cmd_timeout'] > 0.5:
            raise ValueError('cmd_timeout must be <=0.5 seconds')
        if self.p['enable_motion']:
            to_pwm(0, 0, 0, self.p['wheel_radius'], self.p['lx'], self.p['ly'],
                   self.p['pwm_per_rad_s'], self.p['pwm_limit'])
        self.command = (0.0, 0.0, 0.0)
        self.received_at = None
        self.seq = 0
        self.pending = {}
        self.buf = bytearray()
        self.port = None
        self.fault = False
        self.last_ack = time.monotonic()
        try:
            # Set CP210x control lines before opening to avoid holding the
            # ESP32 in reset / bootloader through EN and GPIO0.
            self.port = serial.Serial(port=None, baudrate=115200, timeout=0,
                                      write_timeout=0.05, exclusive=True)
            self.port.dtr = False
            self.port.rts = False
            self.port.port = self.p['port']
            self.port.open()
            self._handshake()
        except Exception:
            self.stop_and_close()
            raise
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.on_cmd, qos)
        self.pub = self.create_publisher(String, '/mecanum/status', 10)
        self.timer = self.create_timer(0.05, self.tick,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('USB ready; enable_motion={}. No encoders or odometry.'
                               .format(self.p['enable_motion']))

    def send(self, pwm):
        seq = self.seq
        self.seq = (seq+1) % 1000000001
        payload = ('P,{},{},{},{},{}\n'.format(seq, *pwm)).encode('ascii')
        n = self.port.write(payload)
        if n != len(payload):
            raise RuntimeError('Incomplete serial write')
        self.pending[seq] = time.monotonic()
        self.pending = {s: t for s, t in self.pending.items()
                        if time.monotonic()-t < 0.5}

    def read_lines(self):
        self.buf.extend(self.port.read(min(self.port.in_waiting, 1024)))
        if len(self.buf) > 4096:
            raise RuntimeError('Serial RX overflow')
        result = []
        while b'\n' in self.buf:
            raw, _, tail = self.buf.partition(b'\n')
            self.buf[:] = tail
            result.append(raw.decode('ascii', errors='replace').strip())
        return result

    def acknowledge(self, line):
        t = feedback(line)
        if t and t[1] in self.pending and t[6] == 0:
            stamp = self.pending.pop(t[1])
            if time.monotonic()-stamp < 0.5:
                self.last_ack = time.monotonic()
                return t
        return None

    def _handshake(self):
        ready = False
        deadline = time.monotonic()+6.0
        next_send = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_send:
                if ready:
                    self.send([0]*4)
                else:
                    self.port.write(b'?\n')
                next_send = now+0.05
            for line in self.read_lines():
                if line == 'READY,MECANUM_PWM_V1':
                    ready = True
                t = self.acknowledge(line)
                if ready and t and t[2:6] == [0]*4:
                    return
            time.sleep(0.005)
        raise RuntimeError('ESP32 identity / zero-command ACK failed')

    def on_cmd(self, msg):
        values = (msg.linear.x, msg.linear.y, msg.linear.z,
                  msg.angular.x, msg.angular.y, msg.angular.z)
        if not all(math.isfinite(x) for x in values):
            self.command = (0.0, 0.0, 0.0)
            self.received_at = None
            self.get_logger().error('Non-finite Twist rejected; stopping')
            return
        # This planar robot only accepts x/y translation and z rotation.
        if any(abs(x) > 1e-9 for x in (msg.linear.z, msg.angular.x, msg.angular.y)):
            self.received_at = None
            self.get_logger().error('Non-planar Twist rejected; stopping')
            return
        limits = (self.p['max_vx'], self.p['max_vy'], self.p['max_wz'])
        self.command = tuple(max(-lim, min(lim, v)) for v, lim in
                             zip((values[0], values[1], values[5]), limits))
        self.received_at = time.monotonic()

    def tick(self):
        if self.fault:
            return
        try:
            for line in self.read_lines():
                if line == 'READY,MECANUM_PWM_V1' or line.startswith('ERROR,'):
                    raise RuntimeError('ESP32 reboot / firmware error')
                self.acknowledge(line)
                if feedback(line):
                    message = String()
                    message.data = line
                    self.pub.publish(message)
            now = time.monotonic()
            if now-self.last_ack > 0.5:
                raise RuntimeError('No fresh ESP32 ACK for 500 ms')
            cmd = select_command(self.command, self.received_at, now,
                                 self.p['cmd_timeout'], self.p['enable_motion'])
            if cmd == (0.0, 0.0, 0.0):
                pwm = [0]*4
            else:
                pwm = to_pwm(*cmd, radius=self.p['wheel_radius'], lx=self.p['lx'],
                             ly=self.p['ly'], gain=self.p['pwm_per_rad_s'],
                             limit=self.p['pwm_limit'])
            self.send(pwm)
        except (OSError, serial.SerialException, RuntimeError, ValueError) as error:
            self.fault = True
            self.received_at = None
            self.get_logger().error('STOP latched: {}. Fix and restart node.'.format(error))
            self.stop_and_close()

    def stop_and_close(self):
        if self.port is None:
            return
        try:
            for _ in range(3):
                self.send([0]*4)
                time.sleep(0.02)
        except (OSError, serial.SerialException):
            pass
        finally:
            self.port.close()
            self.port = None


def main():
    rclpy.init()
    node = None
    try:
        node = MecanumBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop_and_close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
