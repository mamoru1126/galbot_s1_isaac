"""Minimal URDF kinematics for Galbot S1 (numpy only, no ROS dependency).

* KinematicModel  - parses a URDF string, forward kinematics for any link
* ArmIK           - damped least squares IK for a serial chain with joint limits
* Gripper helpers - geometry of the GE103IN gripper (opening <-> joint angle)
"""
import math
import xml.etree.ElementTree as ET

import numpy as np


# --------------------------------------------------------------------------- math
def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def matrix_to_rpy(R):
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def transform(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)):
    T = np.eye(4)
    T[:3, :3] = rpy_to_matrix(*rpy)
    T[:3, 3] = xyz
    return T


def axis_angle_matrix(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    K = np.array([[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]])
    return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)


def quaternion_to_matrix(x, y, z, w):
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)]])


def matrix_to_quaternion(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


def pose_to_matrix(position, quaternion):
    T = np.eye(4)
    T[:3, :3] = quaternion_to_matrix(*quaternion)
    T[:3, 3] = position
    return T


def planar_pose_matrix(x, y, yaw, z=0.0):
    T = np.eye(4)
    c, s = math.cos(yaw), math.sin(yaw)
    T[:3, :3] = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
    T[:3, 3] = (x, y, z)
    return T


def rotation_error(R_current, R_target):
    """Axis-angle vector (world frame) that rotates R_current onto R_target."""
    Re = R_target @ R_current.T
    cos_a = max(-1.0, min(1.0, (np.trace(Re) - 1.0) / 2.0))
    angle = math.acos(cos_a)
    if angle < 1e-9:
        return np.zeros(3)
    v = np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
    s = math.sin(angle)
    if s < 1e-6:  # angle ~ pi
        w, V = np.linalg.eigh((Re + Re.T) / 2.0)
        axis = V[:, np.argmax(w)]
        return axis * angle
    return v / (2.0 * s) * angle


def yaw_of(R):
    return math.atan2(R[1, 0], R[0, 0])


def wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# --------------------------------------------------------------------------- model
class Joint(object):
    __slots__ = ('name', 'type', 'parent', 'child', 'origin', 'axis', 'lower', 'upper',
                 'velocity', 'mimic')

    @property
    def movable(self):
        return self.type in ('revolute', 'continuous', 'prismatic')


class KinematicModel(object):
    def __init__(self, urdf_xml):
        root = ET.fromstring(urdf_xml)
        self.name = root.get('name')
        self.links = [l.get('name') for l in root.findall('link')]
        self.joints = {}
        self.parent_joint = {}
        for j in root.findall('joint'):
            jt = Joint()
            jt.name = j.get('name')
            jt.type = j.get('type')
            jt.parent = j.find('parent').get('link')
            jt.child = j.find('child').get('link')
            o = j.find('origin')
            xyz = [float(v) for v in o.get('xyz', '0 0 0').split()] if o is not None else [0, 0, 0]
            rpy = [float(v) for v in o.get('rpy', '0 0 0').split()] if o is not None else [0, 0, 0]
            jt.origin = transform(xyz, rpy)
            ax = j.find('axis')
            jt.axis = np.array([float(v) for v in ax.get('xyz').split()]) if ax is not None else np.array([1.0, 0, 0])
            lim = j.find('limit')
            if jt.type == 'continuous' or lim is None:
                jt.lower, jt.upper = -1e9, 1e9
            else:
                jt.lower = float(lim.get('lower', '-1e9'))
                jt.upper = float(lim.get('upper', '1e9'))
            jt.velocity = float(lim.get('velocity', '1e9')) if lim is not None else 1e9
            mim = j.find('mimic')
            jt.mimic = (mim.get('joint'), float(mim.get('multiplier', '1')), float(mim.get('offset', '0'))) \
                if mim is not None else None
            self.joints[jt.name] = jt
            self.parent_joint[jt.child] = jt
        children = set(self.parent_joint)
        roots = [l for l in self.links if l not in children]
        self.root = roots[0] if roots else None

    @classmethod
    def from_file(cls, path):
        with open(path) as f:
            return cls(f.read())

    def chain(self, tip, base=None):
        out = []
        link = tip
        while link != base and link in self.parent_joint:
            j = self.parent_joint[link]
            out.append(j)
            link = j.parent
        if base is not None and link != base:
            raise ValueError('%s is not a descendant of %s' % (tip, base))
        return out[::-1]

    def limits(self, name):
        j = self.joints[name]
        return j.lower, j.upper

    def clamp(self, name, value):
        lo, hi = self.limits(name)
        return min(max(value, lo), hi)

    @staticmethod
    def _joint_motion(j, value):
        M = np.eye(4)
        if j.type in ('revolute', 'continuous'):
            M[:3, :3] = axis_angle_matrix(j.axis, value)
        elif j.type == 'prismatic':
            M[:3, 3] = j.axis * value
        return M

    def fk(self, tip, q, base=None):
        T = np.eye(4)
        for j in self.chain(tip, base):
            T = T @ j.origin
            if j.movable:
                T = T @ self._joint_motion(j, q.get(j.name, 0.0))
        return T

    def fk_with_frames(self, tip, q, base=None):
        """Returns (T_base_tip, [(joint, T_base_jointframe)]) for movable joints."""
        T = np.eye(4)
        frames = []
        for j in self.chain(tip, base):
            T = T @ j.origin
            if j.movable:
                frames.append((j, T.copy()))
                T = T @ self._joint_motion(j, q.get(j.name, 0.0))
        return T, frames


# --------------------------------------------------------------------------- IK
class ArmIK(object):
    """Damped least squares IK on a serial chain.

    base_link  : frame the targets are expressed in (e.g. base_footprint)
    tip_link   : last link of the chain (e.g. right_arm_link7)
    joint_names: joints solved for (others in the chain are held at q_fixed)
    """

    def __init__(self, model, base_link, tip_link, joint_names):
        self.model = model
        self.base = base_link
        self.tip = tip_link
        self.names = list(joint_names)
        chain_names = [j.name for j in model.chain(tip_link, base_link) if j.movable]
        missing = [n for n in self.names if n not in chain_names]
        if missing:
            raise ValueError('joints %s not in chain %s -> %s' % (missing, base_link, tip_link))
        self.lower = np.array([model.joints[n].lower for n in self.names])
        self.upper = np.array([model.joints[n].upper for n in self.names])

    def fk(self, q, tcp=None):
        T = self.model.fk(self.tip, q, self.base)
        return T @ tcp if tcp is not None else T

    def solve(self, target, q_seed, q_fixed=None, tcp=None, max_iters=200, pos_tol=1e-3,
              rot_tol=1e-2, rot_weight=0.35, damping=0.03, step_limit=0.25, restarts=6,
              rng_seed=0, posture_gain=0.0, posture=None):
        """Returns (q_dict, ok, pos_err, rot_err). target: 4x4 in base frame (TCP pose)."""
        rng = np.random.default_rng(rng_seed)
        best = None
        seeds = [dict(q_seed)]
        for _ in range(restarts):
            s = dict(q_seed)
            for i, n in enumerate(self.names):
                lo, hi = self.lower[i], self.upper[i]
                span = min(hi - lo, 2 * math.pi)
                s[n] = float(np.clip(q_seed.get(n, 0.0) + rng.normal(0.0, 0.35 * span / 2), lo, hi))
            seeds.append(s)
        for seed in seeds:
            q, ok, ep, er = self._solve_once(target, seed, q_fixed or {}, tcp, max_iters, pos_tol,
                                             rot_tol, rot_weight, damping, step_limit,
                                             posture_gain, posture)
            score = ep + 0.2 * er
            if best is None or (ok and not best[1]) or (ok == best[1] and score < best[4]):
                best = (q, ok, ep, er, score)
            if ok:
                break
        return best[0], best[1], best[2], best[3]

    def _solve_once(self, target, seed, q_fixed, tcp, max_iters, pos_tol, rot_tol, rot_weight,
                    damping, step_limit, posture_gain, posture):
        q = dict(q_fixed)
        q.update({n: float(seed.get(n, 0.0)) for n in self.names})
        idx = {n: i for i, n in enumerate(self.names)}
        ep = er = float('inf')
        for _ in range(max_iters):
            T, frames = self.model.fk_with_frames(self.tip, q, self.base)
            if tcp is not None:
                T = T @ tcp
            e_pos = target[:3, 3] - T[:3, 3]
            e_rot = rotation_error(T[:3, :3], target[:3, :3])
            ep, er = float(np.linalg.norm(e_pos)), float(np.linalg.norm(e_rot))
            if ep < pos_tol and er < rot_tol:
                return q, True, ep, er
            J = np.zeros((6, len(self.names)))
            p_tip = T[:3, 3]
            for j, F in frames:
                if j.name not in idx:
                    continue
                z = F[:3, :3] @ j.axis
                k = idx[j.name]
                if j.type == 'prismatic':
                    J[:3, k] = z
                else:
                    J[:3, k] = np.cross(z, p_tip - F[:3, 3])
                    J[3:, k] = z * rot_weight
            e = np.concatenate([e_pos, e_rot * rot_weight])
            JJt = J @ J.T + (damping ** 2) * np.eye(6)
            dq = J.T @ np.linalg.solve(JJt, e)
            if posture_gain > 0.0 and posture is not None:
                # null-space bias towards a comfortable posture
                Jpinv = J.T @ np.linalg.inv(JJt)
                N = np.eye(len(self.names)) - Jpinv @ J
                qp = np.array([posture.get(n, 0.0) - q[n] for n in self.names])
                dq = dq + posture_gain * (N @ qp)
            m = np.max(np.abs(dq))
            if m > step_limit:
                dq *= step_limit / m
            for n, k in idx.items():
                q[n] = float(np.clip(q[n] + dq[k], self.lower[k] + 1e-4, self.upper[k] - 1e-4))
        return q, False, ep, er


# --------------------------------------------------------------------------- gripper
class GripperGeometry(object):
    """GE103IN gripper: one fixed jaw (part of <side>_gripper_base_link) and one
    active finger on a four-bar (<side>_active_joint1, mimic joints active_joint2 = -q,
    passive_joint = +q).

    Coordinates are in <side>_gripper_base_link: +x is the approach direction (out
    of the fingers), the active finger closes along -z onto the fixed jaw whose
    pad sits at z = fixed_contact[2]. "opening" = pad gap measured along z."""

    # The palm (gripper body between the jaws) ends at x = PALM_X in the gripper
    # base frame: an object top must stay below it (larger x) when grasped.
    PALM_X = 0.094

    def __init__(self, model, side):
        self.model = model
        self.side = side
        self.base = '%s_gripper_base_link' % side
        self.link7 = '%s_arm_link7' % side
        self.j1 = '%s_active_joint1' % side
        self.j2 = '%s_active_joint2' % side
        self.jp = '%s_passive_joint' % side
        self.fixed_contact = model.fk('%s_fixed_contact_link' % side, {}, self.base)[:3, 3]
        self.lower, self.upper = model.limits(self.j1)
        self.T_link7_base = model.fk(self.base, {}, self.link7)
        # Tabulate pad gap vs. joint angle (monotonic decreasing).
        self._angles = np.linspace(self.lower, self.upper, 200)
        self._contacts = np.array([self.active_contact(a) for a in self._angles])
        self._gap = self._contacts[:, 2] - self.fixed_contact[2]

    @property
    def open_angle(self):
        return self.lower

    @property
    def closed_angle(self):
        return self.upper

    def joint_positions(self, angle):
        a = min(max(angle, self.lower), self.upper)
        return {self.j1: a, self.j2: -a, self.jp: a}

    def joint_names(self):
        return [self.j1, self.j2, self.jp]

    def active_contact(self, angle):
        return self.model.fk('%s_active_contact_link' % self.side, self.joint_positions(angle), self.base)[:3, 3]

    def max_opening(self):
        return float(self._gap[0])

    def angle_for_opening(self, opening):
        """Joint angle at which the pad gap equals `opening` (m), clamped."""
        return float(np.interp(opening, self._gap[::-1], self._angles[::-1]))

    def opening_for_angle(self, angle):
        return float(np.interp(angle, self._angles, self._gap))

    def active_contact_for_angle(self, angle):
        return np.array([np.interp(angle, self._angles, self._contacts[:, k]) for k in range(3)])

    def grasp_tcp(self, width, fixed_clearance=0.004, active_clearance=0.003):
        """Where the object centre must be (gripper base frame) to grasp an object of
        `width` (along the closing axis) leaving `fixed_clearance` to the fixed jaw,
        and the finger angle that leaves `active_clearance` on the other side.
        Returns (T_base_tcp 4x4 (rotation = identity), close_angle)."""
        close_angle = self.angle_for_opening(width + fixed_clearance + active_clearance)
        pa = self.active_contact_for_angle(close_angle)
        pf = self.fixed_contact
        T = np.eye(4)
        T[0, 3] = 0.5 * (pa[0] + pf[0])
        T[1, 3] = 0.5 * (pa[1] + pf[1])
        T[2, 3] = pf[2] + fixed_clearance + 0.5 * width
        return T, close_angle

    def tcp_in_link7(self, width, fixed_clearance=0.004, active_clearance=0.003):
        T, _ = self.grasp_tcp(width, fixed_clearance, active_clearance)
        return self.T_link7_base @ T
