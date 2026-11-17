"""Top-down grasp planning for Galbot S1 (IK based, no external planner)."""
import math

import numpy as np

from .kinematics import ArmIK, GripperGeometry
from .robot_config import ARM_JOINTS, ROBOT_BASE_FRAME, POSES, GROUPS


def topdown_rotation(closing_yaw):
    """Gripper base orientation: approach (x) points down, closing axis (z)
    horizontal at `closing_yaw` (direction from the fixed jaw to the finger)."""
    x = np.array([0.0, 0.0, -1.0])
    z = np.array([math.cos(closing_yaw), math.sin(closing_yaw), 0.0])
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


class GraspPlanner(object):
    # Where an object is placed relative to the base for a comfortable grasp
    # (x forward, y towards the arm side) - see docs in README.
    SWEET_SPOT = {'right': (0.62, -0.25), 'left': (0.62, 0.25)}
    # Preferred closing direction (yaw) per arm: fixed jaw outside, finger inside.
    PREFERRED_YAW = {'right': math.pi / 2, 'left': -math.pi / 2}

    def __init__(self, model):
        self.model = model
        self.ik = {s: ArmIK(model, ROBOT_BASE_FRAME, '%s_arm_link7' % s, ARM_JOINTS[s])
                   for s in ('left', 'right')}
        self.gripper = {s: GripperGeometry(model, s) for s in ('left', 'right')}
        self.posture = {}
        for side in ('left', 'right'):
            names = GROUPS['%s_arm' % side][1]
            self.posture[side] = dict(zip(names, POSES['ready']['%s_arm' % side]))

    # ------------------------------------------------------------------ helpers
    def tcp_pose(self, side, q, width=0.04):
        """Current TCP (grasp centre for `width`) in the base frame."""
        g = self.gripper[side]
        return self.ik[side].fk(q, tcp=g.tcp_in_link7(width))

    def gripper_base_pose(self, side, q):
        return self.model.fk('%s_gripper_base_link' % side, q, ROBOT_BASE_FRAME)

    def solve(self, side, T_target, q_current, tcp, restarts=6):
        """IK for a TCP target in the base frame. Returns arm joint list or None."""
        names = ARM_JOINTS[side]
        q, ok, ep, er = self.ik[side].solve(T_target, q_current, q_fixed=q_current, tcp=tcp,
                                            restarts=restarts, posture_gain=0.05,
                                            posture=self.posture[side])
        if not ok:
            return None
        return [q[n] for n in names]

    def cartesian_path(self, side, T_start, T_goal, q_current, tcp, step=0.01):
        """Straight-line TCP path (position interpolation, orientation of T_goal).
        Returns list of arm joint vectors or None if any waypoint fails."""
        names = ARM_JOINTS[side]
        d = np.linalg.norm(T_goal[:3, 3] - T_start[:3, 3])
        n = max(1, int(math.ceil(d / step)))
        q = dict(q_current)
        out = []
        for i in range(1, n + 1):
            T = T_goal.copy()
            T[:3, 3] = T_start[:3, 3] + (T_goal[:3, 3] - T_start[:3, 3]) * (i / float(n))
            sol, ok, ep, er = self.ik[side].solve(T, q, q_fixed=q, tcp=tcp, restarts=2,
                                                  pos_tol=5e-4, rot_tol=5e-3)
            if not ok:
                return None
            vec = [sol[k] for k in names]
            # reject jumps between consecutive waypoints (IK branch switch)
            prev = np.array([q[k] for k in names])
            if np.max(np.abs(np.array(vec) - prev)) > 0.6:
                return None
            q.update(dict(zip(names, vec)))
            out.append(vec)
        return out

    def plan_topdown_grasp(self, side, obj, T_base_obj, q_current, approach_height=0.12,
                           lift_height=0.15, palm_clearance=0.005):
        """Plan pregrasp -> grasp -> lift for an object (SceneObject) at T_base_obj
        (object centre pose in the base frame). Returns dict or None.

        The GE103IN pads are short and the palm is close to the TCP, so the object
        is pinched near its top: the top ends `palm_clearance` below the palm."""
        g = self.gripper[side]
        obj_yaw = math.atan2(T_base_obj[1, 0], T_base_obj[0, 0])
        top_z = T_base_obj[2, 3] + obj.height / 2.0
        for yaw in obj.grasp_yaws(obj_yaw, self.PREFERRED_YAW[side]):
            width = obj.width_along(yaw, obj_yaw)
            if width + 0.010 > g.max_opening():
                continue
            T_tcp_gb, close_angle = g.grasp_tcp(width)
            tcp = g.T_link7_base @ T_tcp_gb
            # TCP x (approach axis, pointing down) relative to the palm
            top_above_tcp = T_tcp_gb[0, 3] - (g.PALM_X + palm_clearance)
            grasp_z = top_z - max(0.0, min(top_above_tcp, obj.height * 0.45))
            R = topdown_rotation(yaw)
            T_grasp = np.eye(4)
            T_grasp[:3, :3] = R
            T_grasp[:3, 3] = [T_base_obj[0, 3], T_base_obj[1, 3], grasp_z]
            T_pre = T_grasp.copy()
            T_pre[2, 3] += approach_height
            pre = self.solve(side, T_pre, q_current, tcp)
            if pre is None:
                continue
            q_pre = dict(q_current)
            q_pre.update(dict(zip(ARM_JOINTS[side], pre)))
            down = self.cartesian_path(side, T_pre, T_grasp, q_pre, tcp)
            if down is None:
                continue
            q_grasp = dict(q_pre)
            q_grasp.update(dict(zip(ARM_JOINTS[side], down[-1])))
            T_lift = T_grasp.copy()
            T_lift[2, 3] += lift_height
            up = self.cartesian_path(side, T_grasp, T_lift, q_grasp, tcp)
            if up is None:
                continue
            return {'yaw': yaw, 'width': width, 'close_angle': close_angle, 'tcp': tcp,
                    'pregrasp': pre, 'descend': down, 'lift': up,
                    'T_grasp': T_grasp, 'T_pre': T_pre, 'T_lift': T_lift}
        return None

    def plan_place(self, side, T_base_place_tcp, q_current, tcp, approach_height=0.12):
        """TCP placing pose in base frame (orientation included) -> pre, descend, retreat."""
        T_pre = T_base_place_tcp.copy()
        T_pre[2, 3] += approach_height
        pre = self.solve(side, T_pre, q_current, tcp)
        if pre is None:
            return None
        q_pre = dict(q_current)
        q_pre.update(dict(zip(ARM_JOINTS[side], pre)))
        down = self.cartesian_path(side, T_pre, T_base_place_tcp, q_pre, tcp)
        if down is None:
            return None
        q_down = dict(q_pre)
        q_down.update(dict(zip(ARM_JOINTS[side], down[-1])))
        up = self.cartesian_path(side, T_base_place_tcp, T_pre, q_down, tcp)
        if up is None:
            return None
        return {'pre': pre, 'descend': down, 'retreat': up}
