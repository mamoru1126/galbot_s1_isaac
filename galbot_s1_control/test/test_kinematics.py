"""Kinematics / grasp planning against the processed URDF."""
import math
import os

import numpy as np

from galbot_s1_control import robot_config as rc
from galbot_s1_control.grasp import GraspPlanner
from galbot_s1_control.kinematics import KinematicModel, planar_pose_matrix
from galbot_s1_control.scene import Scene
from galbot_s1_control.swerve import SwerveKinematics


def ready_q():
    q = {}
    for g, vals in rc.POSES['ready'].items():
        q.update(dict(zip(rc.GROUPS[g][1], vals)))
    return q


def test_model(robot_urdf):
    m = KinematicModel(robot_urdf)
    assert m.root == 'base_footprint'
    for _, (_, joints) in rc.GROUPS.items():
        for j in joints:
            assert j in m.joints
    # base_link sits on the wheels: wheel sphere touches the floor
    z_axle = m.fk('wheel_driving_link1', {}, 'base_footprint')[2, 3]
    assert abs(z_axle - 0.0827) < 2e-3


def test_ready_pose_symmetric(robot_urdf):
    m = KinematicModel(robot_urdf)
    q = ready_q()
    r = m.fk('right_gripper_base_link', q, 'base_footprint')[:3, 3]
    l = m.fk('left_gripper_base_link', q, 'base_footprint')[:3, 3]
    assert abs(r[0] - l[0]) < 1e-3 and abs(r[1] + l[1]) < 1e-3 and abs(r[2] - l[2]) < 1e-3
    assert r[0] > 0.3 and 0.8 < r[2] < 1.3


def test_gripper_geometry(robot_urdf):
    p = GraspPlanner(KinematicModel(robot_urdf))
    for side in ('left', 'right'):
        g = p.gripper[side]
        assert 0.075 < g.max_opening() < 0.09
        prev = None
        for a in np.linspace(g.lower, g.upper, 20):
            o = g.opening_for_angle(a)
            assert prev is None or o < prev
            prev = o
        a = g.angle_for_opening(0.05)
        assert abs(g.opening_for_angle(a) - 0.05) < 1e-3


def test_grasp_all_objects(robot_urdf, src):
    scene = Scene.from_file(os.path.join(src, "galbot_s1_isaac", "config", "scene.yaml"))
    p = GraspPlanner(KinematicModel(robot_urdf))
    q = ready_q()
    for obj in scene.objects:
        for side in ('left', 'right'):
            sx, sy = p.SWEET_SPOT[side]
            T = planar_pose_matrix(sx, sy, obj.yaw, scene.object_spawn_z(obj))
            plan = p.plan_topdown_grasp(side, obj, T, q)
            assert plan is not None, (obj.name, side)
            qg = dict(q)
            qg.update(dict(zip(rc.ARM_JOINTS[side], plan['descend'][-1])))
            tcp = p.ik[side].fk(qg, tcp=plan['tcp'])
            assert np.linalg.norm(tcp[:2, 3] - T[:2, 3]) < 2e-3


def test_swerve():
    s = SwerveKinematics(rc.WHEEL_POSITIONS, 0.0827)
    for twist in ((0.3, 0.0, 0.0), (0.0, 0.2, 0.0), (0.1, -0.1, 0.4), (0.0, 0.0, -0.5)):
        a, w = s.inverse(*twist)
        back = s.forward(a, w)
        assert np.allclose(back, twist, atol=1e-6)
        assert all(abs(x) <= 2.9 for x in a)
