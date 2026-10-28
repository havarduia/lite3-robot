#!/usr/bin/env python3
"""Live mounting-angle readout for the Livox MID360, from its built-in IMU.

At rest an accelerometer measures the 'up' direction, so gravity gives absolute
pitch and roll with no drift. Watch this while you physically adjust the mount.

    lidar-angle            # target the recommended 15 deg nose-down
    lidar-angle 20         # target some other angle
    lidar-angle 0          # level it up instead

The vector is normalised, so it works whether the driver reports g or m/s^2.
"""
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

TARGET = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
TOL = 1.0          # deg, "on target" window
N = 100            # samples averaged (~0.5 s at 200 Hz)

C_OK, C_WARN, C_BAD, C_DIM, C_R = (
    '\033[92m', '\033[93m', '\033[91m', '\033[2m', '\033[0m')


def bar(val, lo, hi, width=41, mark=None):
    """Horizontal gauge with a centre tick and an optional target mark."""
    cells = ['-'] * width
    cells[width // 2] = '|'
    if mark is not None:
        m = int((mark - lo) / (hi - lo) * (width - 1))
        if 0 <= m < width:
            cells[m] = 'T'
    p = int((val - lo) / (hi - lo) * (width - 1))
    p = max(0, min(width - 1, p))
    cells[p] = '#'
    return ''.join(cells)


class AngleProbe(Node):
    def __init__(self):
        super().__init__('lidar_angle')
        self.acc = []
        self.gyr = []
        self.create_subscription(Imu, '/livox/imu', self.cb, 50)

    def cb(self, m):
        a = m.linear_acceleration
        g = m.angular_velocity
        self.acc.append([a.x, a.y, a.z])
        self.gyr.append([g.x, g.y, g.z])
        self.acc = self.acc[-N:]
        self.gyr = self.gyr[-N:]

    def angles(self):
        """Return (pitch_down, roll, steady, spin) or None until warmed up."""
        if len(self.acc) < N:
            return None
        a = np.mean(np.array(self.acc), axis=0)
        n = np.linalg.norm(a)
        if n < 1e-6:
            return None
        ax, ay, az = a / n

        # +pitch = nose down; +roll = right side down.
        pitch = math.degrees(math.atan2(ax, math.hypot(ay, az)))
        roll = math.degrees(math.atan2(-ay, az))

        spin = float(np.linalg.norm(np.mean(np.array(self.gyr), axis=0)))
        wobble = float(np.std(np.linalg.norm(np.array(self.acc), axis=1) / n))
        steady = spin < 0.05 and wobble < 0.02
        return pitch, roll, steady, spin


def main():
    rclpy.init()
    node = AngleProbe()
    print(f"\n  Livox MID360 angle meter   target {TARGET:+.0f}° nose-down"
          f"   (Ctrl-C to quit)\n")
    try:
        first = True
        while True:
            rclpy.spin_once(node, timeout_sec=1.0)
            r = node.angles()
            if r is None:
                continue
            pitch, roll, steady, spin = r

            err = pitch - TARGET
            if abs(err) <= TOL:
                pc, verdict = C_OK, 'ON TARGET'
            elif abs(err) <= 3:
                pc, verdict = C_WARN, ('tilt DOWN %.1f°' % -err if err < 0
                                       else 'tilt UP %.1f°' % err)
            else:
                pc, verdict = C_BAD, ('tilt DOWN %.1f°' % -err if err < 0
                                      else 'tilt UP %.1f°' % err)

            if abs(roll) <= 1.0:
                rc, rv = C_OK, 'level'
            else:
                rc, rv = (C_WARN if abs(roll) <= 3 else C_BAD), (
                    'RIGHT side down' if roll > 0 else 'LEFT side down')

            motion = (f"{C_DIM}steady{C_R}" if steady
                      else f"{C_WARN}MOVING - hold still{C_R}")

            if not first:
                sys.stdout.write('\033[5A')
            first = False
            print(f"  pitch {pc}{pitch:+7.2f}°{C_R}  "
                  f"[{bar(pitch, -45, 45, mark=TARGET)}]  {pc}{verdict:<16}{C_R}")
            print(f"  roll  {rc}{roll:+7.2f}°{C_R}  "
                  f"[{bar(roll, -45, 45, mark=0)}]  {rc}{rv:<16}{C_R}")
            print(f"  {motion}   {C_DIM}spin {spin:.3f} rad/s"
                  f"   T = target, | = centre{C_R}          ")
            print(f"  {C_DIM}+pitch = nose down   +roll = right side down{C_R}"
                  "                    ")
            print()
    except KeyboardInterrupt:
        print("\n  stopped.\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
