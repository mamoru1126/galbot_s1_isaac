"""Runs backend_isaac.py against stand-ins of the Isaac Sim python modules.

This does not test Isaac Sim (no GPU in CI). It executes every code path of the
backend - stage construction with the real pxr USD bindings, camera mounting,
DOF bookkeeping, physics callbacks, attach / detach, images - so that mistakes in
our own code show up before somebody starts Isaac Sim. The stubs only implement the
calls the backend makes, with the signatures of Isaac Sim 6.2.
"""
import collections
import enum
import math
import sys
import types

import numpy as np
import pytest

try:
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
except ImportError:
    Usd = None

pytestmark = pytest.mark.skipif(Usd is None, reason='pxr not installed (pip install usd-core)')


# ---------------------------------------------------------------------------------- stubs
class _State(object):
    stage = None
    time = 0.0
    playing = False
    callbacks = {}
    next_id = 0
    articulations = []


S = _State()


class SimulationEvent(enum.Enum):
    PHYSICS_PRE_STEP = 'pre'
    PHYSICS_POST_STEP = 'post'


class SimulationManager(object):
    @classmethod
    def setup_simulation(cls, dt=None, device=None):
        cls.dt = dt

    @classmethod
    def get_simulation_time(cls):
        return S.time

    @classmethod
    def register_callback(cls, cb, event, order=0):
        S.next_id += 1
        S.callbacks[S.next_id] = (event, cb)
        return S.next_id

    @classmethod
    def deregister_callback(cls, uid):
        return S.callbacks.pop(uid, None) is not None


def _world(prim):
    return np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T


def _quat_wxyz(R):
    from galbot_isaac import geometry as geo
    return geo.xyzw_to_wxyz(geo.matrix_to_quat(R))


class RigidPrim(object):
    def __init__(self, paths):
        self.path = paths
        self.prim = S.stage.GetPrimAtPath(paths)
        assert self.prim.IsValid(), paths
        self.gravity = True

    def get_world_poses(self, indices=None):
        T = _world(self.prim)
        return np.array([T[:3, 3]]), np.array([_quat_wxyz(T[:3, :3])])

    def set_world_poses(self, positions=None, orientations=None, indices=None):
        positions = np.asarray(positions, dtype=float)
        orientations = np.asarray(orientations, dtype=float)
        assert positions.shape == (1, 3) and orientations.shape == (1, 4)
        w, x, y, z = orientations[0]
        xf = UsdGeom.Xformable(self.prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*positions[0]))
        xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, Gf.Vec3d(x, y, z)))

    def set_velocities(self, linear_velocities=None, angular_velocities=None, indices=None):
        assert np.asarray(linear_velocities).shape == (1, 3)

    def set_enabled_gravities(self, enabled, indices=None):
        self.gravity = bool(enabled)


class Articulation(object):
    def __init__(self, paths):
        root = S.stage.GetPrimAtPath(paths)
        assert root.HasAPI(UsdPhysics.ArticulationRootAPI)
        self.dof_names = [p.GetName() for p in Usd.PrimRange(root)
                          if p.IsA(UsdPhysics.RevoluteJoint) or p.IsA(UsdPhysics.PrismaticJoint)]
        n = len(self.dof_names)
        self.q, self.qd = np.zeros((1, n)), np.zeros((1, n))
        self.qt, self.vt = np.zeros((1, n)), np.zeros((1, n))
        self.calls = collections.Counter()
        S.articulations.append(self)

    def _put(self, arr, values, dof_indices):
        values = np.asarray(values, dtype=float)
        idx = list(range(arr.shape[1])) if dof_indices is None else list(dof_indices)
        assert values.shape == (1, len(idx)), (values.shape, len(idx))
        arr[0, idx] = values[0]

    def set_dof_gains(self, stiffnesses=None, dampings=None, indices=None, dof_indices=None):
        self._put(np.zeros_like(self.q), stiffnesses, dof_indices)
        self.calls['gains'] += 1

    def set_dof_max_efforts(self, max_efforts, indices=None, dof_indices=None):
        self._put(np.zeros_like(self.q), max_efforts, dof_indices)

    def set_dof_positions(self, positions, indices=None, dof_indices=None):
        self._put(self.q, positions, dof_indices)

    def set_dof_velocities(self, velocities, indices=None, dof_indices=None):
        self._put(self.qd, velocities, dof_indices)

    def set_dof_position_targets(self, positions, indices=None, dof_indices=None):
        self._put(self.qt, positions, dof_indices)

    def set_dof_velocity_targets(self, velocities, indices=None, dof_indices=None):
        self._put(self.vt, velocities, dof_indices)

    def get_dof_positions(self, indices=None, dof_indices=None):
        return self.q.copy()

    def get_dof_velocities(self, indices=None, dof_indices=None):
        return self.qd.copy()

    def physics_step(self):
        self.qd = (self.qt - self.q) * 120.0
        self.q = self.qt.copy()


class App(object):
    def __init__(self):
        self.frames = 0

    def update(self):
        self.frames += 1
        if not S.playing:
            return
        for _ in range(2):
            for event, cb in list(S.callbacks.values()):
                if event is SimulationEvent.PHYSICS_PRE_STEP:
                    cb(1.0 / 120.0, None)
            for a in S.articulations:
                a.physics_step()
            S.time += 1.0 / 120.0
            for event, cb in list(S.callbacks.values()):
                if event is SimulationEvent.PHYSICS_POST_STEP:
                    cb(1.0 / 120.0, None)

    def is_running(self):
        return True


class _Texture(object):
    def __init__(self):
        self.enabled = []

    def set_updates_enabled(self, enabled):
        self.enabled.append(enabled)


class _RenderProduct(object):
    def __init__(self, path, resolution):
        assert S.stage.GetPrimAtPath(str(path)).IsA(UsdGeom.Camera), path
        self.resolution = resolution
        self.hydra_texture = _Texture()


class _Annotator(object):
    def attach(self, rp):
        self.rp = rp

    def get_data(self, do_array_copy=False):
        w, h = self.rp.resolution
        return np.full((h, w, 4), 128, dtype=np.uint8)

    def detach(self):
        pass


def install_stubs(monkeypatch):
    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, m)
        return m

    def create_new_stage(template=None):
        S.stage = Usd.Stage.CreateInMemory()
        return S.stage

    def add_reference_to_stage(usd_path, path, prim_type='Xform', variants=None):
        prim = S.stage.DefinePrim(path, prim_type)
        prim.GetReferences().AddReference(usd_path)
        return prim

    def play(commit=True):
        S.playing = True

    def stop(commit=True):
        S.playing = False

    for name in ('isaacsim', 'isaacsim.core', 'isaacsim.core.experimental', 'isaacsim.core.experimental.utils',
                 'omni'):
        mod(name)
    mod('isaacsim.core.experimental.utils.app', play=play, stop=stop, is_playing=lambda: S.playing,
        enable_extension=lambda name, enabled=True: True, update_app=lambda steps=1, callback=None: None)
    mod('isaacsim.core.experimental.utils.stage', create_new_stage=create_new_stage,
        get_current_stage=lambda backend=None: S.stage, add_reference_to_stage=add_reference_to_stage)
    mod('isaacsim.core.experimental.prims', Articulation=Articulation, RigidPrim=RigidPrim)
    mod('isaacsim.core.simulation_manager', SimulationManager=SimulationManager, SimulationEvent=SimulationEvent)
    mod('isaacsim.core.rendering_manager',
        ViewportManager=types.SimpleNamespace(set_camera_view=lambda *a, **k: None))
    rep = mod('omni.replicator')
    core = mod('omni.replicator.core',
               create=types.SimpleNamespace(render_product=lambda path, res: _RenderProduct(path, res)),
               AnnotatorRegistry=types.SimpleNamespace(get_annotator=lambda name: _Annotator()))
    rep.core = core


def fake_robot_usd(path, urdf_xml):
    """What the URDF importer + asset transformer produce, reduced to what the backend reads:
    nested link Xforms with RigidBodyAPI, joints, an articulation root and a Physics variant set."""
    from galbot_s1_control.kinematics import KinematicModel
    from galbot_isaac import usd_scene
    model = KinematicModel(urdf_xml)
    stage = Usd.Stage.CreateNew(path)
    root = UsdGeom.Xform.Define(stage, '/galbot_s1_isaac').GetPrim()
    stage.SetDefaultPrim(root)
    vs = root.GetVariantSets().AddVariantSet('Physics')
    for v in ('none', 'physx', 'mujoco'):
        vs.AddVariant(v)
    paths = {model.root: '/galbot_s1_isaac/' + model.root}
    order = [model.root]
    for link in model.links:
        if link not in paths:
            chain = [j.child for j in model.chain(link)]
            for c in chain:
                if c not in paths:
                    paths[c] = paths[model.parent_joint[c].parent] + '/' + c
                    order.append(c)
    for link in order:
        prim = UsdGeom.Xform.Define(stage, paths[link]).GetPrim()
        j = model.parent_joint.get(link)
        usd_scene.set_pose(prim, j.origin if j is not None else np.eye(4))
        UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.ArticulationRootAPI.Apply(root)
    UsdGeom.Scope.Define(stage, '/galbot_s1_isaac/joints')
    for j in model.joints.values():
        p = '/galbot_s1_isaac/joints/' + j.name
        if j.type in ('revolute', 'continuous'):
            UsdPhysics.RevoluteJoint.Define(stage, p)
        elif j.type == 'prismatic':
            UsdPhysics.PrismaticJoint.Define(stage, p)
        else:
            UsdPhysics.FixedJoint.Define(stage, p)
    stage.GetRootLayer().Save()
    return path


# ---------------------------------------------------------------------------------- test
@pytest.fixture
def backend(monkeypatch, tmp_path, settings, scene, isaac_urdf):
    install_stubs(monkeypatch)
    from galbot_s1_control import kinematics
    from galbot_s1_control import robot_config as rc
    from galbot_isaac import backend_isaac, robot_import
    usd = fake_robot_usd(str(tmp_path / 'robot.usda'), isaac_urdf)
    monkeypatch.setattr(robot_import, 'ensure_robot_usd', lambda *a, **k: usd)
    logs = []
    b = backend_isaac.IsaacBackend(App(), settings, scene, rc, kinematics, headless=False, log=logs.append)
    b.start()
    b.logs = logs
    yield b
    b.close()
    S.callbacks.clear()
    S.articulations.clear()
    S.time, S.playing = 0.0, False


def test_start(backend, scene):
    b = backend
    assert any('robot variant Physics: physx' in l for l in b.logs)
    sel = b.stage.GetPrimAtPath(b.robot_path).GetVariantSets().GetVariantSet('Physics').GetVariantSelection()
    assert sel == 'physx'
    assert {'base_footprint', 'right_arm_link7', 'head_camera_link'} <= set(b.link_paths)
    assert b.dof_names and set(b.joint_list).isdisjoint({'base_x_joint', 'base_y_joint', 'base_yaw_joint'})
    # cameras below their links, render products created
    head = b.stage.GetPrimAtPath(b.cams['head_camera']['path'])
    assert head.IsA(UsdGeom.Camera) and head.GetParent().GetName() == 'head_camera_link'
    assert set(b.camera_specs()) == {'head_camera', 'right_wrist_camera', 'left_wrist_camera', 'overview_camera'}
    # robot links without gravity, PhysX articulation settings on the root
    from galbot_isaac import usd_scene
    if usd_scene.PhysxSchema is not None:      # not part of usd-core
        link = b.stage.GetPrimAtPath(b.link_paths['right_arm_link7'])
        assert link.GetAttribute('physxRigidBody:disableGravity').Get() is True
    # initial pose = ready pose, base at the spawn pose
    names, q, _ = b.joint_state()
    from galbot_s1_control import robot_config as rc
    for j, v in rc.INITIAL_POSITIONS.items():
        assert abs(q[names.index(j)] - v) < 1e-9
    assert b.base_state()[:3] == pytest.approx(tuple(scene.robot_spawn))


def test_base_command_integrates_per_physics_step(backend):
    b = backend
    b.set_base_command(0.2, 0.0, 0.0)
    t0 = b.sim_time()
    for _ in range(60):
        b.step()
    assert b.sim_time() - t0 == pytest.approx(1.0)
    x, y, yaw, vx, vy, wz = b.base_state()
    assert x == pytest.approx(0.2, abs=1e-6) and abs(y) < 1e-9
    b.set_base_command(0.0, 0.0, math.pi / 2)
    for _ in range(60):
        b.step()
    assert b.base_state()[2] == pytest.approx(math.pi / 2, abs=1e-6)
    b.set_base_command(0.1, 0.0, 0.0)          # now driving along world +y
    for _ in range(60):
        b.step()
    x, y = b.base_state()[:2]
    assert x == pytest.approx(0.2, abs=1e-6) and y == pytest.approx(0.1, abs=1e-6)
    b.set_base_command(0.0, 0.0, 0.0)
    b.reset_base(0.0, 0.0, 0.0)
    b.step()
    assert b.base_state()[:3] == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)


def test_joint_targets_and_wheels(backend):
    b = backend
    b.set_joint_targets({'right_arm_joint1': (0.5, 0.2), 'base_x_joint': (9.0, 0.0)})
    b.set_wheel_velocities({'wheel_driving_joint1': 3.0})
    b.step()
    names, q, _ = b.joint_state()
    assert q[names.index('right_arm_joint1')] == pytest.approx(0.5)
    art = S.articulations[-1]
    assert art.vt[0, b.idx['wheel_driving_joint1']] == 3.0
    assert b.base_state()[0] == pytest.approx(0.0, abs=1e-9)   # planar joints are not set this way


def test_attach_follow_detach(backend, scene):
    b = backend
    p0, q0 = b.object_pose('red_can')
    assert b.attach('red_can', 'right_arm_link7')
    assert b.objects['red_can'].gravity is False
    # move the link (the stub physics does not move USD links, do it by hand) -> object follows
    link = b.stage.GetPrimAtPath(b.link_paths['right_arm_link7'])
    T = _world(link)
    UsdGeom.Xformable(link).AddTranslateOp(UsdGeom.XformOp.PrecisionDouble, 'lift').Set(Gf.Vec3d(0, 0, 0.1))
    T2 = _world(link)
    assert np.linalg.norm((T2 - T)[:3, 3]) == pytest.approx(0.1, abs=1e-9)
    b.step()
    p1, _ = b.object_pose('red_can')
    assert np.linalg.norm((T2 - T)[:3, 3] - (p1 - p0)) < 1e-6
    assert b.detach('red_can') and not b.detach('red_can')
    assert b.objects['red_can'].gravity is True
    b.set_object_pose('red_can', (1.0, 0.0, 0.9), (0, 0, 0, 1))
    np.testing.assert_allclose(b.object_pose('red_can')[0], [1.0, 0.0, 0.9])


def test_attach_joint_mode(backend):
    b = backend
    b.attach_mode = 'joint'
    assert b.attach('blue_box', 'left_arm_link7')
    j = UsdPhysics.FixedJoint(b.stage.GetPrimAtPath('/World/grasp_joints/blue_box'))
    assert [str(t) for t in j.GetBody0Rel().GetTargets()] == [b.link_paths['left_arm_link7']]
    assert b.detach('blue_box')
    assert not b.stage.GetPrimAtPath('/World/grasp_joints/blue_box').IsValid()


def test_images(backend):
    b = backend
    b.set_camera_active('head_camera', False)
    b.set_camera_active('head_camera', True)
    img = b.camera_image('head_camera')
    assert img.shape == (480, 640, 3) and img.dtype == np.uint8 and img.flags['C_CONTIGUOUS']
    assert b.camera_image('overview_camera').shape == (480, 640, 3)
    assert b.camera_image('nope') is None


def test_with_sim_core(backend, settings):
    """The real simulation loop on top of the stubbed backend."""
    from galbot_s1_control import robot_config as rc
    from galbot_isaac.sim_core import SimCore
    sys.path.insert(0, __file__.rsplit('/', 1)[0])
    from test_sim_core import FakeRos
    ros = FakeRos()
    core = SimCore(backend, ros, dict(settings, realtime_factor=0), 'galbot_s1', rc, log=lambda *_: None)
    ros.command('torso_controller', [0.5])
    ros.request('attach', 'galbot_s1 right_arm_link7 red_can')
    for _ in range(30):
        core.step()
    assert ros.out['joints'][-1]['torso_lift_joint1'] == pytest.approx(0.5)
    assert ros.out['attached'][-1] == {'red_can': 'right_arm_link7'}
    assert ros.out['img_head_camera'] and ros.out['clock'][-1] == pytest.approx(0.5)
