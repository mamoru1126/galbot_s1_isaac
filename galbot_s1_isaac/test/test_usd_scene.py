"""USD authoring of the world with the real pxr bindings (pip install usd-core; skipped otherwise)."""
import math

import numpy as np
import pytest

pxr = pytest.importorskip('pxr')
from pxr import Gf, Usd, UsdGeom, UsdPhysics  # noqa: E402

from galbot_isaac import geometry as geo  # noqa: E402
from galbot_isaac import usd_scene  # noqa: E402


@pytest.fixture
def stage(scene, settings):
    st = Usd.Stage.CreateInMemory()
    paths = usd_scene.build(st, scene, settings)
    st.paths = paths
    return st


def world_matrix(prim):
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return np.array(m).T   # Gf is row-vector convention


def test_physics_scene_and_ground(stage):
    scene = UsdPhysics.Scene(stage.GetPrimAtPath(usd_scene.PHYSICS_SCENE))
    assert scene and abs(scene.GetGravityMagnitudeAttr().Get() - 9.81) < 1e-6
    ground = stage.GetPrimAtPath('/World/ground')
    assert ground.HasAPI(UsdPhysics.CollisionAPI)
    assert UsdGeom.GetStageUpAxis(stage) == UsdGeom.Tokens.z


def test_table_and_tray_height(stage, scene):
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    top = cache.ComputeWorldBound(stage.GetPrimAtPath('/World/table/top')).ComputeAlignedRange()
    assert abs(top.GetMax()[2] - scene.table_height) < 1e-6
    tray = cache.ComputeWorldBound(stage.GetPrimAtPath('/World/tray')).ComputeAlignedRange()
    assert abs(tray.GetMax()[2] - scene.tray_top()) < 1e-6
    assert abs(tray.GetMin()[2] - scene.table_height) < 1e-6


def test_objects(stage, scene):
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    for o in scene.objects:
        prim = stage.GetPrimAtPath(stage.paths[o.name])
        assert prim.HasAPI(UsdPhysics.RigidBodyAPI)
        assert abs(UsdPhysics.MassAPI(prim).GetMassAttr().Get() - o.mass) < 1e-9
        geom = prim.GetChild('geom')
        assert geom.HasAPI(UsdPhysics.CollisionAPI)
        T = world_matrix(prim)
        assert np.allclose(T[:3, 3], [o.position[0], o.position[1], scene.object_spawn_z(o) + 0.001])
        rng = cache.ComputeWorldBound(geom).ComputeAlignedRange()
        assert abs(rng.GetMin()[2] - (scene.table_height + 0.001)) < 1e-6      # resting on the table
        assert abs((rng.GetMax()[0] - rng.GetMin()[0]) - o.size[0]) < 1e-6


def test_overview_camera_looks_at_the_table(stage, scene):
    cam = stage.GetPrimAtPath('/World/overview_camera')
    T = world_matrix(cam)
    forward = T[:3, :3] @ np.array([0.0, 0.0, -1.0])        # USD camera looks along -Z
    up = T[:3, :3] @ np.array([0.0, 1.0, 0.0])
    target = np.array([0.7, 0.0, 0.7])                       # see the comment in scene.yaml
    to_target = (target - T[:3, 3]) / np.linalg.norm(target - T[:3, 3])
    assert forward @ to_target > 0.97
    assert up[2] > 0.5                                       # not upside down
    c = UsdGeom.Camera(cam)
    hfov = 2 * math.atan(c.GetHorizontalApertureAttr().Get() / (2 * c.GetFocalLengthAttr().Get()))
    assert abs(hfov - scene.raw['overview_camera']['hfov']) < 1e-6


def test_robot_camera_pose_on_link(stage):
    # camera body pose with an offset and a yaw: the USD camera must look along the body x axis
    T = geo.make_pose((0.1, 0.0, 0.2), geo.rpy_to_matrix(0.0, 0.3, 0.5))
    prim = usd_scene.camera(stage, '/World/test_cam', T, 640, 480, 1.2, 0.03, 20.0)
    W = world_matrix(prim)
    np.testing.assert_allclose(W[:3, :3] @ np.array([0, 0, -1.0]), T[:3, 0], atol=1e-9)
    np.testing.assert_allclose(W[:3, 3], T[:3, 3], atol=1e-12)


def test_fixed_joint(stage):
    T = geo.make_pose((0.01, 0.02, 0.15), geo.rpy_to_matrix(0.1, 0.2, 0.3))
    prim = usd_scene.fixed_joint(stage, '/World/grasp_joints/red_can', '/World/robot/link7',
                                 '/World/objects/red_can', T)
    j = UsdPhysics.FixedJoint(prim)
    assert [str(p) for p in j.GetBody1Rel().GetTargets()] == ['/World/objects/red_can']
    q = j.GetLocalRot0Attr().Get()
    R = np.array(Gf.Matrix3d(Gf.Rotation(Gf.Quatd(q.GetReal(), Gf.Vec3d(q.GetImaginary()))))).T
    np.testing.assert_allclose(R, T[:3, :3], atol=1e-6)
    assert j.GetExcludeFromArticulationAttr().Get() is True
