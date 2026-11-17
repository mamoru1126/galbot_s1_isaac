"""Swerve (4 x steer + drive) wheel kinematics of the Galbot S1 omni chassis."""
import math


class SwerveKinematics(object):
    def __init__(self, wheel_positions, wheel_radius, steer_limit=2.9, min_speed=1e-3):
        self.positions = list(wheel_positions)
        self.radius = wheel_radius
        self.steer_limit = steer_limit
        self.min_speed = min_speed
        self.steer = [0.0] * len(self.positions)

    def inverse(self, vx, vy, wz):
        """Body twist -> (steering angles [rad], wheel angular velocities [rad/s]).

        Each module picks the steering angle closest to its current one (a wheel
        can roll backwards instead of turning by 180 deg). When the base is not
        moving the steering angles are kept."""
        angles = []
        speeds = []
        for i, (x, y) in enumerate(self.positions):
            wx = vx - wz * y
            wy = vy + wz * x
            v = math.hypot(wx, wy)
            current = self.steer[i]
            if v < self.min_speed:
                angles.append(current)
                speeds.append(0.0)
                continue
            target = math.atan2(wy, wx)
            best = None
            for cand, sign in ((target, 1.0), (target + math.pi, -1.0), (target - math.pi, -1.0),
                               (target + 2 * math.pi, 1.0), (target - 2 * math.pi, 1.0)):
                if abs(cand) > self.steer_limit:
                    continue
                cost = abs(cand - current)
                if best is None or cost < best[0]:
                    best = (cost, cand, sign)
            _, ang, sign = best
            angles.append(ang)
            speeds.append(sign * v / self.radius)
        self.steer = angles
        return angles, speeds

    def forward(self, angles, wheel_speeds):
        """Least squares body twist from module states (for odometry from wheels)."""
        import numpy as np
        A = []
        b = []
        for (x, y), a, w in zip(self.positions, angles, wheel_speeds):
            v = w * self.radius
            A.append([1.0, 0.0, -y])
            b.append(v * math.cos(a))
            A.append([0.0, 1.0, x])
            b.append(v * math.sin(a))
        sol, *_ = np.linalg.lstsq(np.array(A), np.array(b), rcond=None)
        return float(sol[0]), float(sol[1]), float(sol[2])
