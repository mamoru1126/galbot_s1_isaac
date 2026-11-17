"""USD authoring of the demo world (plain pxr API, Isaac Sim only).

Builds from scene.yaml: physics scene, ground, lights, table, tray, the graspable
objects (rigid bodies, frame = geometric centre like in Gazebo) and the fixed
overview camera. Robot cameras are created by backend_isaac.py on the robot links.
"""
import numpy as np
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade

from . import geometry as geo

try:  # PhysX specific attributes (solver, time steps); optional
    from pxr import PhysxSchema
except ImportError:  # pragma: no cover
    PhysxSchema = None

WORLD = '/World'
LOOKS = '/World/Looks'
PHYSICS_SCENE = '/physicsScene'
OBJECTS = '/World/objects'


def _quatd(R):
    x, y, z, w = geo.matrix_to_quat(R)
    return Gf.Quatd(float(w), Gf.Vec3d(float(x), float(y), float(z)))


def set_pose(prim, T, scale=None):
    """translate / orient / (scale) xform ops, the layout Isaac Sim expects."""
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*[float(v) for v in T[:3, 3]]))
    xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(_quatd(T[:3, :3]))
    if scale is not None:
        xf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*[float(v) for v in scale]))


def preview_material(stage, name, color, roughness=0.5, metallic=0.0):
    path = '%s/%s' % (LOOKS, name)
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + '/Shader')
    sh.CreateIdAttr('UsdPreviewSurface')
    sh.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*[float(c) for c in color]))
    sh.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(float(roughness))
    sh.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(float(metallic))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), 'surface')
    return mat


def physics_material(stage, name, friction, restitution=0.0):
    mat = UsdShade.Material.Define(stage, '%s/%s' % (LOOKS, name))
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr(float(friction))
    api.CreateDynamicFrictionAttr(float(friction))
    api.CreateRestitutionAttr(float(restitution))
    return mat


def bind(prim, visual=None, physics=None):
    api = UsdShade.MaterialBindingAPI.Apply(prim)
    if visual is not None:
        api.Bind(visual)
    if physics is not None:
        api.Bind(physics, UsdShade.Tokens.weakerThanDescendants, 'physics')


def box(stage, path, center, size, visual=None, physics=None, collide=True):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
    set_pose(cube.GetPrim(), geo.make_pose(center), scale=size)
    if collide:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    bind(cube.GetPrim(), visual, physics if collide else None)
    return cube.GetPrim()


def cylinder(stage, path, radius, height, visual=None, physics=None):
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.CreateRadiusAttr(float(radius))
    cyl.CreateHeightAttr(float(height))
    cyl.CreateAxisAttr(UsdGeom.Tokens.z)
    cyl.CreateExtentAttr([Gf.Vec3f(-radius, -radius, -height / 2.0), Gf.Vec3f(radius, radius, height / 2.0)])
    UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
    bind(cyl.GetPrim(), visual, physics)
    return cyl.GetPrim()


def physics_scene(stage, settings):
    ph = settings['physics']
    scene = UsdPhysics.Scene.Define(stage, PHYSICS_SCENE)
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr(9.81)
    if PhysxSchema is not None:
        api = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        gpu = str(ph.get('device', 'cpu')).startswith('cuda')
        api.CreateTimeStepsPerSecondAttr(int(round(1.0 / float(ph['dt']))))
        api.CreateSolverTypeAttr(str(ph.get('solver', 'TGS')))
        api.CreateEnableGPUDynamicsAttr(gpu)
        api.CreateBroadphaseTypeAttr('GPU' if gpu else 'MBP')
        api.CreateEnableCCDAttr(False)
    return scene.GetPrim()


def camera(stage, path, T_world_body, width, height, hfov, near, far):
    """USD camera from a ROS-style camera body pose (x forward, z up)."""
    cam = UsdGeom.Camera.Define(stage, path)
    ha = 20.955
    cam.CreateHorizontalApertureAttr(ha)
    cam.CreateVerticalApertureAttr(ha * float(height) / float(width))
    cam.CreateFocalLengthAttr(geo.focal_length_for_hfov(float(hfov), ha))
    cam.CreateClippingRangeAttr(Gf.Vec2f(float(near), float(far)))
    T = np.array(T_world_body, dtype=float).copy()
    T[:3, :3] = T[:3, :3] @ geo.ROS_BODY_TO_USD_CAMERA
    set_pose(cam.GetPrim(), T)
    return cam.GetPrim()


def build(stage, scene, settings):
    """Author the world. Returns {object name: rigid body prim path}."""
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, WORLD).GetPrim()
    stage.SetDefaultPrim(world)
    physics_scene(stage, settings)

    # lights
    dome = UsdLux.DomeLight.Define(stage, WORLD + '/DomeLight')
    dome.CreateIntensityAttr(700.0)
    dome.CreateColorAttr(Gf.Vec3f(0.92, 0.95, 1.0))
    sun = UsdLux.DistantLight.Define(stage, WORLD + '/SunLight')
    sun.CreateIntensityAttr(2500.0)
    sun.CreateAngleAttr(1.0)
    set_pose(sun.GetPrim(), geo.make_pose((0, 0, 10), geo.rpy_to_matrix(0.6, 0.35, 0.4)))

    # ground
    floor_vis = preview_material(stage, 'floor', (0.55, 0.55, 0.52), roughness=0.8)
    floor_phys = physics_material(stage, 'floor_physics', 1.0)
    box(stage, WORLD + '/ground', (0.0, 0.0, -0.01), (40.0, 40.0, 0.02), floor_vis, floor_phys)

    # table (static colliders, same parts as the Gazebo world)
    t = scene.raw['table']
    dx, dy, h = [float(v) for v in t['size']]
    th = float(t.get('top_thickness', 0.04))
    cx, cy = [float(v) for v in t['center']]
    wood = preview_material(stage, 'table', (0.62, 0.45, 0.30), roughness=0.6)
    table_phys = physics_material(stage, 'table_physics', 1.0)
    table = UsdGeom.Xform.Define(stage, WORLD + '/table').GetPrim()
    set_pose(table, geo.make_pose((cx, cy, 0.0)))
    leg = 0.05
    box(stage, WORLD + '/table/top', (0, 0, h - th / 2.0), (dx, dy, th), wood, table_phys)
    lx, ly = dx / 2.0 - leg, dy / 2.0 - leg
    for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        box(stage, WORLD + '/table/leg%d' % i, (sx * lx, sy * ly, (h - th) / 2.0), (leg, leg, h - th),
            wood, table_phys)

    if scene.tray:
        sx, sy, sz = scene.tray['size']
        tx, ty = scene.tray['center']
        tray_vis = preview_material(stage, 'tray', scene.tray['color'], roughness=0.4)
        box(stage, WORLD + '/tray', (tx, ty, h + sz / 2.0), (sx, sy, sz), tray_vis, table_phys)

    # graspable objects: rigid body Xform (frame = centre) + one collider child
    UsdGeom.Scope.Define(stage, OBJECTS)
    obj_phys = physics_material(stage, 'object_physics', 1.0)
    paths = {}
    for o in scene.objects:
        path = '%s/%s' % (OBJECTS, o.name)
        xf = UsdGeom.Xform.Define(stage, path).GetPrim()
        z = scene.object_spawn_z(o) + 0.001
        set_pose(xf, geo.make_pose((o.position[0], o.position[1], z), geo.rpy_to_matrix(0, 0, o.yaw)))
        UsdPhysics.RigidBodyAPI.Apply(xf)
        UsdPhysics.MassAPI.Apply(xf).CreateMassAttr(float(o.mass))
        vis = preview_material(stage, 'object_' + o.name, o.color, roughness=0.35)
        if o.shape == 'cylinder':
            cylinder(stage, path + '/geom', o.radius, o.length, vis, obj_phys)
        else:
            box(stage, path + '/geom', (0, 0, 0), o.size, vis, obj_phys)
        paths[o.name] = path

    # fixed overview camera (web UI "俯瞰")
    ov = scene.raw.get('overview_camera')
    if ov and settings['cameras'].get('enabled', True):
        x, y, z, roll, pitch, yaw = [float(v) for v in ov['pose']]
        camera(stage, WORLD + '/overview_camera', geo.make_pose((x, y, z), geo.rpy_to_matrix(roll, pitch, yaw)),
               int(ov.get('width', 640)), int(ov.get('height', 480)), float(ov.get('hfov', 1.1)),
               settings['cameras']['near'], settings['cameras']['far'])
    return paths


def disable_gravity(prims):
    """PhysX per-body gravity switch (ideal gravity compensation of the robot links)."""
    if PhysxSchema is None:
        return 0
    n = 0
    for prim in prims:
        PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr(True)
        n += 1
    return n


def articulation_solver(prim, position_iterations, velocity_iterations):
    if PhysxSchema is None:
        return
    api = PhysxSchema.PhysxArticulationAPI.Apply(prim)
    api.CreateSolverPositionIterationCountAttr(int(position_iterations))
    api.CreateSolverVelocityIterationCountAttr(int(velocity_iterations))
    api.CreateEnabledSelfCollisionsAttr(False)


def fixed_joint(stage, path, body0, body1, T_body0_body1):
    """Fixed joint keeping body1 at T_body0_body1 relative to body0 (attach_mode: joint)."""
    joint = UsdPhysics.FixedJoint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    p = T_body0_body1[:3, 3]
    x, y, z, w = geo.matrix_to_quat(T_body0_body1[:3, :3])
    joint.CreateLocalPos0Attr(Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])))
    joint.CreateLocalRot0Attr(Gf.Quatf(float(w), Gf.Vec3f(float(x), float(y), float(z))))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
    joint.CreateExcludeFromArticulationAttr(True)
    return joint.GetPrim()
