"""Joint space trajectories (minimum-jerk timing along a piecewise linear path)."""
import numpy as np


def min_jerk(s):
    s = min(max(s, 0.0), 1.0)
    return s * s * s * (10.0 - 15.0 * s + 6.0 * s * s)


class JointPath(object):
    """Moves through `waypoints` (list of position vectors) starting at `start`.

    The path is parametrised by normalised joint-space arc length and the
    parameter follows a minimum-jerk profile, so the motion starts and stops
    smoothly and dense Cartesian waypoints give a straight tool path."""

    def __init__(self, start, waypoints, max_velocity, t0, min_duration=0.3, duration=None):
        pts = [np.asarray(start, dtype=float)] + [np.asarray(w, dtype=float) for w in waypoints]
        self.points = np.array(pts)
        seg = np.diff(self.points, axis=0)
        seg_len = np.linalg.norm(seg, axis=1)
        self.cum = np.concatenate([[0.0], np.cumsum(seg_len)])
        self.total = float(self.cum[-1])
        vmax = np.broadcast_to(np.asarray(max_velocity, dtype=float), self.points.shape[1:])
        per_joint = np.sum(np.abs(seg), axis=0) if len(seg) else np.zeros(self.points.shape[1])
        # peak velocity of a min-jerk profile is 1.875 x the mean velocity
        needed = float(np.max(per_joint / np.maximum(vmax, 1e-6))) * 1.875 if len(seg) else 0.0
        self.duration = max(min_duration, needed) if duration is None else max(duration, needed)
        self.t0 = t0
        self.goal = self.points[-1].copy()

    def sample(self, t):
        if self.total < 1e-9:
            return self.goal.copy()
        s = min_jerk((t - self.t0) / self.duration) * self.total
        i = int(np.searchsorted(self.cum, s, side='right')) - 1
        i = min(max(i, 0), len(self.points) - 2)
        seg = self.cum[i + 1] - self.cum[i]
        a = 0.0 if seg < 1e-12 else (s - self.cum[i]) / seg
        return self.points[i] + a * (self.points[i + 1] - self.points[i])

    def finished(self, t):
        return t - self.t0 >= self.duration
