"""Checks of everything the Isaac Sim app relies on that can run without Isaac Sim."""
import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from galbot_s1_control import robot_config as rc
from galbot_s1_control.kinematics import KinematicModel
from galbot_isaac import config, geometry as geo
from galbot_isaac.config import PLANAR_JOINTS
from galbot_isaac.robot_import import drive_overrides


def test_isaac_urdf_structure(isaac_urdf):
    root = ET.fromstring(isaac_urdf)
    m = KinematicModel(isaac_urdf)
    assert m.root == 'isaac_world'
    chain = [j.name for j in m.chain('base_footprint')]
    assert chain == list(PLANAR_JOINTS)
    assert [m.joints[j].type for j in chain] == ['prismatic', 'prismatic', 'continuous']
    assert root.find('.//mimic') is None
    for link in root.findall('link'):
        assert link.find('inertial') is not None, link.get('name')
        if link.get('name').startswith(('wheel_steering_link', 'wheel_driving_link')):
            assert link.find('collision') is None
    for mesh in root.iter('mesh'):
        assert mesh.get('filename').startswith('package://galbot_s1_description/meshes/')
        rel = mesh.get('filename')[len('package://galbot_s1_description/'):]
        assert os.path.isfile(os.path.join(os.path.dirname(config.PKG_DIR), 'galbot_s1_description', rel)), rel


def test_isaac_urdf_matches_description(isaac_urdf, desc_urdf):
    a, b = KinematicModel(desc_urdf), KinematicModel(isaac_urdf)
    assert set(a.links) <= set(b.links)
    q = dict(rc.INITIAL_POSITIONS)
    for link in a.links:
        np.testing.assert_allclose(a.fk(link, q), b.fk(link, q, base='base_footprint'), atol=1e-9)
    q.update(base_x_joint=1.2, base_y_joint=-0.4, base_yaw_joint=0.7)
    np.testing.assert_allclose(b.fk('base_footprint', q), geo.planar_pose(1.2, -0.4, 0.7), atol=1e-9)


def test_drive_settings(settings, isaac_urdf):
    m = KinematicModel(isaac_urdf)
    movable = [n for n, j in m.joints.items() if j.movable]
    for n in movable:
        d = config.drive_for_joint(settings, n)
        assert d['mode'] == ('velocity' if n in rc.WHEEL_JOINTS else 'position'), n
        if d['mode'] == 'position':
            assert d['stiffness'] > 0 and d['damping'] > 0, n
    assert config.drive_for_joint(settings, 'base_x_joint')['stiffness'] >= 1e5
    assert config.drive_for_joint(settings, 'right_active_joint1')['max_effort'] <= 50
    target, stiffness, damping = drive_overrides(settings, isaac_urdf)
    assert set(target) == {'^%s$' % n for n in movable}
    assert target['^wheel_driving_joint1$'] == 'velocity'
    assert stiffness['^wheel_driving_joint1$'] == 0.0


def test_camera_links(settings, isaac_urdf):
    m = KinematicModel(isaac_urdf)
    for name, cam in settings['cameras']['robot'].items():
        assert cam['link'] in m.links, name
        assert 0.3 < float(cam['hfov']) < 2.5


def test_camera_axes():
    # USD camera looks along its -Z axis: after the conversion that must be the ROS body +X
    R = np.eye(3) @ geo.ROS_BODY_TO_USD_CAMERA
    np.testing.assert_allclose(R @ np.array([0, 0, -1.0]), [1, 0, 0], atol=1e-12)
    np.testing.assert_allclose(R @ np.array([0, 1.0, 0]), [0, 0, 1], atol=1e-12)   # image up = body z
    np.testing.assert_allclose(R @ np.array([1.0, 0, 0]), [0, -1, 0], atol=1e-12)  # image right = body -y
    assert abs(np.linalg.det(geo.ROS_BODY_TO_USD_CAMERA) - 1.0) < 1e-12
    f = geo.focal_length_for_hfov(1.2, 20.955)
    assert abs(2 * math.atan(20.955 / (2 * f)) - 1.2) < 1e-12


@pytest.mark.parametrize('rpy', [(0, 0, 0), (0.3, -0.2, 2.5), (math.pi, 0.1, -1.0), (0.0, 1.2, 3.1)])
def test_quaternions(rpy):
    R = geo.rpy_to_matrix(*rpy)
    q = geo.matrix_to_quat(R)
    np.testing.assert_allclose(geo.quat_to_matrix(q), R, atol=1e-12)
    np.testing.assert_allclose(geo.wxyz_to_xyzw(geo.xyzw_to_wxyz(q)), q)


def test_scene_fits(scene, settings):
    # objects on the table, overview camera present, robot clear of the table
    tx, ty = scene.table_center
    dx, dy, _ = scene.table_size
    for o in scene.objects:
        assert abs(o.position[0] - tx) < dx / 2 and abs(o.position[1] - ty) < dy / 2
    assert scene.raw['overview_camera']['width'] > 0
    assert scene.table_front_x > 0.6
