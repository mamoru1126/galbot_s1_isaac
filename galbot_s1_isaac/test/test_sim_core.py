"""SimCore + kinematic backend without ROS (a fake RosIO records everything)."""
import collections
import math

import numpy as np
import pytest

from galbot_s1_control import kinematics
from galbot_s1_control import robot_config as rc
from galbot_isaac.backend_kinematic import KinematicBackend
from galbot_isaac.sim_core import SimCore, controller_table, support_height


class FakeRos(object):
    def __init__(self):
        self.commands, self.seq = {}, collections.Counter()
        self.base, self.base_seq = None, 0
        self.requests = []
        self.out = collections.defaultdict(list)
        self.images_published = collections.Counter()
        self.images_dropped = 0

    # inputs
    def command(self, ctrl, values):
        self.commands[ctrl] = list(values)
        self.seq[ctrl] += 1

    def base_cmd(self, vx, vy, wz):
        self.base = (vx, vy, wz)
        self.base_seq += 1

    def request(self, kind, text):
        self.requests.append((kind, text))

    def snapshot(self):
        r, self.requests = self.requests, []
        return dict(self.commands), dict(self.seq), self.base, self.base_seq, r

    # outputs
    def info(self, t):
        self.out['log'].append(t)

    warn = info

    def publish_clock(self, t):
        self.out['clock'].append(t)

    def publish_joints(self, t, names, pos, vel):
        self.out['joints'].append(dict(zip(names, pos)))

    def publish_base(self, t, p, q, lin, ang):
        self.out['base'].append((p, q, lin, ang))

    def publish_object(self, t, name, p, q):
        self.out['obj_' + name].append(np.array(p))

    def publish_attached(self, a):
        self.out['attached'].append(dict(a))

    def camera_wanted(self, name):
        return True

    def submit_image(self, t, name, rgb):
        self.out['img_' + name].append(rgb.shape)


@pytest.fixture
def sim(settings, scene, desc_urdf):
    settings['realtime_factor'] = 0
    backend = KinematicBackend(settings, scene, desc_urdf, rc, kinematics, log=lambda *_: None)
    backend.start()
    ros = FakeRos()
    core = SimCore(backend, ros, settings, scene.robot_name, rc, log=lambda *_: None)
    return core, backend, ros


def run(core, seconds):
    t0 = core.backend.sim_time()
    while core.backend.sim_time() - t0 < seconds:
        core.step()


def test_controllers_cover_all_groups():
    table = controller_table(rc)
    for ctrl, joints in rc.GROUPS.values():
        assert table[ctrl] == (list(joints), 'position')
    assert table['wheel_controller'][1] == 'velocity'


def test_initial_pose_and_publishing(sim):
    core, backend, ros = sim
    run(core, 1.0)
    js = ros.out['joints'][-1]
    for j, v in rc.INITIAL_POSITIONS.items():
        assert abs(js[j] - v) < 1e-9
    assert not any(n.startswith('base_') for n in js)
    assert 55 <= len(ros.out['joints']) <= 61          # 60 Hz
    assert 18 <= len(ros.out['obj_red_can']) <= 21      # 20 Hz
    assert ros.out['img_overview_camera'] and ros.out['img_head_camera']
    assert ros.out['clock'] == sorted(ros.out['clock'])


def test_arm_command_and_limits(sim):
    core, backend, ros = sim
    q = list(rc.INITIAL_POSITIONS[j] for j in rc.ARM_JOINTS['right'])
    q[0] += 0.3
    q[5] = 5.0                                           # beyond the limit -> clamped
    ros.command('right_arm_controller', q)
    ros.command('torso_controller', [0.4])
    run(core, 2.0)
    js = ros.out['joints'][-1]
    assert abs(js['right_arm_joint1'] - q[0]) < 1e-6
    assert abs(js['right_arm_joint6'] - backend.model.joints['right_arm_joint6'].upper) < 1e-6
    assert abs(js['torso_lift_joint1'] - 0.4) < 1e-6
    ros.command('torso_controller', [0.1, 0.2])        # wrong length -> ignored with a warning
    run(core, 0.5)
    assert abs(ros.out['joints'][-1]['torso_lift_joint1'] - 0.4) < 1e-6
    assert any('expected 1 values' in t for t in ros.out['log'])


def test_feedforward_velocity(sim):
    core, backend, ros = sim
    q = list(rc.INITIAL_POSITIONS[j] for j in rc.ARM_JOINTS['left'])
    for k in range(10):                                  # 0.5 rad/s ramp at ~60 Hz
        q[0] += 0.5 / 60.0
        ros.command('left_arm_controller', q)
        core.step()
    assert abs(core.ff['left_arm_joint1'] - 0.5) < 0.05
    run(core, 0.3)                                       # no new command -> feed-forward released
    assert core.ff['left_arm_joint1'] == 0.0


def test_base_velocity_and_timeout(sim):
    core, backend, ros = sim
    ros.base_cmd(0.2, 0.0, 0.0)
    run(core, 1.0)
    x, y, yaw = backend.base[:3]
    assert 0.1 < x < 0.13 and abs(y) < 1e-9             # 0.2 m/s for 0.5 s (timeout) + stop
    p, q, lin, ang = ros.out['base'][-1]
    assert lin[0] == 0.0
    ros.base_cmd(0.0, 0.0, 0.5)
    run(core, 0.4)
    assert backend.base[2] > 0.15


def test_attach_detach_reset(sim, scene):
    core, backend, ros = sim
    can = scene.by_name['red_can']
    ros.request('attach', 'galbot_s1 right_arm_link7 red_can')
    ros.request('attach', 'other_robot right_arm_link7 blue_box')      # unknown model -> warning
    run(core, 0.1)
    assert core.attached == {'red_can': 'right_arm_link7'}
    assert ros.out['attached'][-1] == {'red_can': 'right_arm_link7'}
    p0 = ros.out['obj_red_can'][-1].copy()
    ros.command('torso_controller', [0.3])                              # lift 0.1 m -> object follows
    run(core, 1.5)
    assert abs(ros.out['obj_red_can'][-1][2] - p0[2] - 0.1) < 1e-3
    ros.request('detach', 'red_can')
    run(core, 0.2)
    assert core.attached == {} and ros.out['attached'][-1] == {}
    p = ros.out['obj_red_can'][-1]
    assert abs(p[2] - (support_height(scene, p[0], p[1]) + can.height / 2)) < 1e-6   # dropped
    ros.request('set_model_pose', 'red_can 1.45 -0.35 %.4f 0 0 0' % scene.object_spawn_z(can))
    ros.request('set_model_pose', 'galbot_s1 0.5 0.2 0 0 0 1.0')
    run(core, 0.1)
    np.testing.assert_allclose(ros.out['obj_red_can'][-1], [1.45, -0.35, scene.object_spawn_z(can)], atol=1e-6)
    assert backend.base[:3] == pytest.approx([0.5, 0.2, 1.0], abs=0.02)


def test_support_height(scene):
    assert support_height(scene, 0.0, 0.0) == 0.0
    assert support_height(scene, 1.6, 0.4) == pytest.approx(scene.table_height)
    tx, ty = scene.tray['center']
    assert support_height(scene, tx, ty) == pytest.approx(scene.tray_top())
    assert math.isclose(scene.tray_top(), 0.758)
