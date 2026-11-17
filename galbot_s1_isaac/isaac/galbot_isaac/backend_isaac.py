"""Isaac Sim 6.x backend (PhysX articulation + RTX cameras).

* robot  : config/../urdf/galbot_s1_isaac.urdf imported with the Isaac URDF importer
           (cached USD), fixed root + planar base joints (x, y, yaw) with stiff drives,
           every other joint position-driven (wheels velocity-driven), link gravity off
* world  : built from scene.yaml with plain USD (usd_scene.py)
* grasp  : attach_mode follow (object put on the gripper pose each physics step) or
           joint (PhysX fixed joint created at run time)
* images : replicator render products + "rgb" annotators, read only while somebody
           subscribes to /galbot/<camera>/image_raw

Requires a running SimulationApp (created in run_sim.py before importing this module).
"""
import math

import numpy as np

from . import config
from . import geometry as geo
from .config import PLANAR_JOINTS

GRASP_JOINTS = '/World/grasp_joints'
ROBOT_CAMERAS = '/World/robot_cameras'


class IsaacBackend(object):
    def __init__(self, app, settings, scene, rc, kinematics, headless=False, rebuild_usd=False, log=print):
        self.app = app
        self.settings = settings
        self.scene = scene
        self.rc = rc
        self.headless = headless
        self.rebuild_usd = rebuild_usd
        self.log = log
        self.urdf_path = config.isaac_urdf_file()
        with open(self.urdf_path) as f:
            self.model = kinematics.KinematicModel(f.read())
        self.robot_path = settings['robot']['prim_path']
        self.attach_mode = settings['robot'].get('attach_mode', 'follow')
        self.wheel_joints = set(rc.WHEEL_JOINTS)

        cams = settings['cameras']
        self.cam_specs = {}
        self.cam_links = {}
        if cams.get('enabled', True):
            for name, c in cams.get('robot', {}).items():
                self.cam_specs[name] = (int(c['width']), int(c['height']), float(c['hfov']), float(c['rate']))
                self.cam_links[name] = c['link']
            ov = scene.raw.get('overview_camera')
            if ov:
                self.cam_specs['overview_camera'] = (int(ov.get('width', 640)), int(ov.get('height', 480)),
                                                     float(ov.get('hfov', 1.1)), float(cams.get('overview_rate', 10)))

        self.base_cmd = (0.0, 0.0, 0.0)
        self.base_target = None
        self.held = {}            # object -> (link, T_link_obj)
        self.grasp_joints = {}    # object -> joint prim path (attach_mode joint)
        self.cams = {}            # name -> {'rp', 'annot', 'path', 'follow': (link, T_link_cam) | None}
        self._cb_ids = []
        self._cb_errors = set()
        self._q = self._qd = None
        self._obj_cache = {}
        self.link_rigid = {}

    # ================================================================== start-up
    def start(self):
        import isaacsim.core.experimental.utils.app as app_utils
        import isaacsim.core.experimental.utils.stage as stage_utils
        from isaacsim.core.experimental.prims import Articulation, RigidPrim
        from isaacsim.core.simulation_manager import SimulationManager
        from pxr import Usd, UsdPhysics

        from . import robot_import, usd_scene
        self._app_utils = app_utils
        self._RigidPrim = RigidPrim
        self._SM = SimulationManager
        self._usd_scene = usd_scene

        s = self.settings
        usd = robot_import.ensure_robot_usd(s, self.urdf_path, config.description_dir(), config.cache_dir(s),
                                            rebuild=self.rebuild_usd, log=self.log)

        stage_utils.create_new_stage()
        self.app.update()
        stage = self.stage = stage_utils.get_current_stage(backend='usd')
        self.object_paths = usd_scene.build(stage, self.scene, s)
        robot_prim = stage_utils.add_reference_to_stage(usd_path=usd, path=self.robot_path)
        self._select_physics_variant(robot_prim)
        self.app.update()

        # robot prims: rigid links and the articulation root
        robot_prim = stage.GetPrimAtPath(self.robot_path)
        self.link_paths, root = {}, None
        for prim in Usd.PrimRange(robot_prim):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                self.link_paths.setdefault(prim.GetName(), str(prim.GetPath()))
            if root is None and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                root = prim
        if root is None:
            raise RuntimeError('no ArticulationRootAPI below %s (import failed?)' % self.robot_path)
        missing = [l for l in ('base_footprint', 'left_arm_link7', 'right_arm_link7') if l not in self.link_paths]
        self.log('robot: %d rigid links, articulation root %s%s' % (
            len(self.link_paths), root.GetPath(), ', MISSING %s' % missing if missing else ''))
        usd_scene.articulation_solver(root, s['physics']['position_iterations'], s['physics']['velocity_iterations'])
        if s['robot'].get('disable_gravity', True):
            usd_scene.disable_gravity([stage.GetPrimAtPath(p) for p in self.link_paths.values()])

        self._create_cameras(stage)

        try:
            SimulationManager.setup_simulation(dt=float(s['physics']['dt']), device=s['physics']['device'])
        except Exception as exc:
            self.log('setup_simulation: %s - falling back to set_physics_dt' % exc)
            SimulationManager.set_physics_sim_device(s['physics']['device'])
            SimulationManager.set_physics_dt(float(s['physics']['dt']))

        self.robot = Articulation(str(root.GetPath()))
        self.objects = {name: RigidPrim(path) for name, path in self.object_paths.items()}
        for side in ('left', 'right'):
            self._link_rigid('%s_arm_link7' % side)
        self._create_render_products()
        if not self.headless:
            self._set_viewport()

        app_utils.play()
        for _ in range(3):
            self.app.update()
        self._init_robot()
        self._register_callbacks()
        for _ in range(5):
            self.app.update()
        self.log('Isaac Sim ready: physics dt %.4f s on %s, %d DOFs, attach_mode %s, cameras %s'
                 % (float(s['physics']['dt']), s['physics']['device'], len(self.dof_names), self.attach_mode,
                    sorted(self.cams)))

    def _select_physics_variant(self, prim):
        """Assets written by the Isaac asset transformer keep their physics in a "Physics"
        variant set whose default is empty: select the PhysX variant."""
        sets = prim.GetVariantSets()
        for set_name in sets.GetNames():
            if set_name.lower() != 'physics':
                continue
            vs = sets.GetVariantSet(set_name)
            names = vs.GetVariantNames()
            pick = next((n for n in names if n.lower() == 'physx'), None)
            if pick:
                vs.SetVariantSelection(pick)
            self.log('robot variant %s: %s (available %s)' % (set_name, pick, list(names)))

    def _set_viewport(self):
        try:
            from isaacsim.core.rendering_manager import ViewportManager
            ViewportManager.set_camera_view('/OmniverseKit_Persp', eye=[-1.4, -2.6, 2.3], target=[0.9, 0.0, 0.6])
        except Exception as exc:
            self.log('viewport camera not set: %s' % exc)

    def _mount(self, link):
        """Nearest rigid link (the link itself or a fixed-joint ancestor) and T_mount_link."""
        T = np.eye(4)
        cur = link
        while cur not in self.link_paths:
            j = self.model.parent_joint.get(cur)
            if j is None or j.movable:
                raise RuntimeError('no rigid body prim for link %s' % link)
            T = j.origin @ T
            cur = j.parent
        return cur, T

    def _create_cameras(self, stage):
        cfg = self.settings['cameras']
        for name, link in self.cam_links.items():
            w, h, hfov, _rate = self.cam_specs[name]
            mount, T = self._mount(link)
            parent = stage.GetPrimAtPath(self.link_paths[mount])
            if parent.IsInstance() or parent.IsInstanceProxy():
                path = '%s/%s' % (ROBOT_CAMERAS, name)          # pose updated every frame
                self._usd_scene.camera(stage, path, np.eye(4), w, h, hfov, cfg['near'], cfg['far'])
                follow = (mount, T)
            else:
                path = '%s/%s' % (self.link_paths[mount], name)  # rigidly attached child prim
                self._usd_scene.camera(stage, path, T, w, h, hfov, cfg['near'], cfg['far'])
                follow = None
            self.cams[name] = {'path': path, 'follow': follow}
            self.log('camera %s on %s%s' % (name, mount, ' (follow)' if follow else ''))
        if 'overview_camera' in self.cam_specs:
            self.cams['overview_camera'] = {'path': '/World/overview_camera', 'follow': None}

    def _create_render_products(self):
        if not self.cams:
            return
        import omni.replicator.core as rep
        for name, cam in self.cams.items():
            w, h = self.cam_specs[name][:2]
            rp = rep.create.render_product(cam['path'], (w, h))
            # no hydra texture updates while the annotator graph is being built (same as
            # isaacsim.replicator.experimental.mobility_gen), then on again
            self._set_updates(rp, False)
            annot = rep.AnnotatorRegistry.get_annotator('rgb')
            annot.attach(rp)
            self._set_updates(rp, True)
            cam['rp'], cam['annot'] = rp, annot
            if cam['follow']:
                self._link_rigid(cam['follow'][0])

    def _init_robot(self):
        r = self.robot
        self.dof_names = list(r.dof_names)
        self.idx = {n: i for i, n in enumerate(self.dof_names)}
        missing = [j for j in PLANAR_JOINTS if j not in self.idx]
        if missing:
            raise RuntimeError('planar base joints missing in the articulation: %s' % missing)
        self.base_idx = [self.idx[j] for j in PLANAR_JOINTS]
        self.joint_list = [n for n in self.dof_names if n not in PLANAR_JOINTS]
        self.joint_idx = [self.idx[n] for n in self.joint_list]
        n = len(self.dof_names)

        stiff, damp = np.zeros((1, n)), np.zeros((1, n))
        eff_idx, eff_val = [], []
        for name, i in self.idx.items():
            d = config.drive_for_joint(self.settings, name)
            stiff[0, i], damp[0, i] = float(d['stiffness']), float(d['damping'])
            if d.get('max_effort'):
                eff_idx.append(i)
                eff_val.append(float(d['max_effort']))
        r.set_dof_gains(stiffnesses=stiff, dampings=damp)
        if eff_idx:
            r.set_dof_max_efforts(np.array([eff_val]), dof_indices=eff_idx)

        q0 = np.zeros((1, n))
        x, y, yaw = self.scene.robot_spawn
        q0[0, self.base_idx] = (x, y, yaw)
        for j, v in self.rc.INITIAL_POSITIONS.items():
            if j in self.idx:
                q0[0, self.idx[j]] = v
        r.set_dof_positions(q0)
        r.set_dof_velocities(np.zeros((1, n)))
        r.set_dof_position_targets(q0)
        r.set_dof_velocity_targets(np.zeros((1, n)))
        self.q_target = q0.copy()
        self.v_target = np.zeros((1, n))
        self.base_target = [float(x), float(y), float(yaw)]
        self._q, self._qd = q0[0].copy(), np.zeros(n)
        self.initial = {j: float(q0[0, self.idx[j]]) for j in self.joint_list if j not in self.wheel_joints}

    def _register_callbacks(self):
        try:
            from isaacsim.core.simulation_manager import SimulationEvent
            pre, post = SimulationEvent.PHYSICS_PRE_STEP, SimulationEvent.PHYSICS_POST_STEP
        except ImportError:  # older 6.x releases
            from isaacsim.core.simulation_manager import IsaacEvents
            pre, post = IsaacEvents.PRE_PHYSICS_STEP, IsaacEvents.POST_PHYSICS_STEP
        self._cb_ids.append(self._SM.register_callback(self._pre_physics, pre))
        self._cb_ids.append(self._SM.register_callback(self._post_physics, post))

    def _cb_error(self, where, exc):
        if where not in self._cb_errors:
            self._cb_errors.add(where)
            self.log('error in %s callback (reported once): %r' % (where, exc))

    # ================================================================== physics callbacks
    def _pre_physics(self, dt, _context=None):
        """Integrate the commanded base velocity with the exact physics step."""
        try:
            if self.base_target is None or not self._app_utils.is_playing():
                return
            vx, vy, wz = self.base_cmd
            x, y, yaw = self.base_target
            c, s = math.cos(yaw), math.sin(yaw)
            vxw, vyw = c * vx - s * vy, s * vx + c * vy
            x, y, yaw = x + vxw * dt, y + vyw * dt, yaw + wz * dt
            if self._q is not None:   # anti wind-up: never run away from the real base
                ax, ay, ayaw = self._q[self.base_idx]
                x = ax + max(-0.05, min(0.05, x - ax))
                y = ay + max(-0.05, min(0.05, y - ay))
                yaw = ayaw + max(-0.1, min(0.1, yaw - ayaw))
            self.base_target = [x, y, yaw]
            self.robot.set_dof_position_targets(np.array([[x, y, yaw]]), dof_indices=self.base_idx)
            self.robot.set_dof_velocity_targets(np.array([[vxw, vyw, wz]]), dof_indices=self.base_idx)
        except Exception as exc:
            self._cb_error('pre-physics', exc)

    def _post_physics(self, dt, _context=None):
        """attach_mode follow: keep held objects on the gripper."""
        if not self.held or self.attach_mode != 'follow':
            return
        try:
            for name, (link, T_lo) in self.held.items():
                T = self._link_pose(link) @ T_lo
                p, q = geo.pose_to_pq(T)
                self.objects[name].set_world_poses(positions=np.array([p]), orientations=np.array([geo.xyzw_to_wxyz(q)]))
                self.objects[name].set_velocities(linear_velocities=np.zeros((1, 3)), angular_velocities=np.zeros((1, 3)))
        except Exception as exc:
            self._cb_error('post-physics', exc)

    # ================================================================== helpers
    def _link_rigid(self, link):
        if link not in self.link_rigid:
            if link not in self.link_paths:
                raise KeyError('unknown link %s' % link)
            self.link_rigid[link] = self._RigidPrim(self.link_paths[link])
        return self.link_rigid[link]

    @staticmethod
    def _np(a):
        return a.numpy() if hasattr(a, 'numpy') else np.asarray(a)

    def _link_pose(self, link):
        pos, quat = self._link_rigid(link).get_world_poses()
        return geo.pose_from_pq(self._np(pos)[0], geo.wxyz_to_xyzw(self._np(quat)[0]))

    def _object_T(self, name):
        pos, quat = self.objects[name].get_world_poses()
        return geo.pose_from_pq(self._np(pos)[0], geo.wxyz_to_xyzw(self._np(quat)[0]))

    # ================================================================== backend API
    def is_running(self):
        return self.app.is_running()

    def close(self):
        for cid in self._cb_ids:
            try:
                self._SM.deregister_callback(cid)
            except Exception:
                pass
        for cam in self.cams.values():
            try:
                cam['annot'].detach()
                cam['rp'].destroy()
            except Exception:
                pass
        try:
            self._app_utils.stop()
        except Exception:
            pass

    def sim_time(self):
        return float(self._SM.get_simulation_time())

    def step(self):
        if self.cams:
            self._update_follow_cameras()
        if self._app_utils.is_playing():
            try:
                self.robot.set_dof_position_targets(self.q_target[:, self.joint_idx], dof_indices=self.joint_idx)
                self.robot.set_dof_velocity_targets(self.v_target[:, self.joint_idx], dof_indices=self.joint_idx)
            except Exception as exc:
                self._cb_error('set targets', exc)
        self.app.update()
        if self._app_utils.is_playing():
            try:
                self._q = self._np(self.robot.get_dof_positions())[0].copy()
                self._qd = self._np(self.robot.get_dof_velocities())[0].copy()
            except Exception as exc:
                self._cb_error('read state', exc)
        return self.sim_time()

    def _update_follow_cameras(self):
        for cam in self.cams.values():
            if not cam.get('follow'):
                continue
            try:
                link, T_lc = cam['follow']
                T = self._link_pose(link) @ T_lc
                T[:3, :3] = T[:3, :3] @ geo.ROS_BODY_TO_USD_CAMERA
                self._usd_scene.set_pose(self.stage.GetPrimAtPath(cam['path']), T)
            except Exception as exc:
                self._cb_error('follow camera', exc)

    def joint_names(self):
        return list(self.joint_list)

    def joint_limits(self):
        out = {}
        for n in self.joint_list:
            j = self.model.joints.get(n)
            if j is not None:
                out[n] = (j.lower, j.upper, j.velocity)
        return out

    def initial_positions(self):
        return dict(self.initial)

    def set_joint_targets(self, targets):
        for n, (p, v) in targets.items():
            i = self.idx.get(n)
            if i is not None and n not in PLANAR_JOINTS:
                self.q_target[0, i] = p
                self.v_target[0, i] = v

    def set_wheel_velocities(self, velocities):
        for n, v in velocities.items():
            i = self.idx.get(n)
            if i is not None:
                self.v_target[0, i] = v

    def set_base_command(self, vx, vy, wz):
        self.base_cmd = (float(vx), float(vy), float(wz))

    def reset_base(self, x, y, yaw):
        q = self._q.copy()
        q[self.base_idx] = (x, y, yaw)
        self.robot.set_dof_positions(np.array([[x, y, yaw]]), dof_indices=self.base_idx)
        self.robot.set_dof_velocities(np.zeros((1, 3)), dof_indices=self.base_idx)
        self.base_target = [x, y, yaw]
        self._q = q

    def joint_state(self):
        return self.joint_list, self._q[self.joint_idx], self._qd[self.joint_idx]

    def base_state(self):
        x, y, yaw = self._q[self.base_idx]
        vx, vy, wz = self._qd[self.base_idx]
        return float(x), float(y), geo.wrap_angle(float(yaw)), float(vx), float(vy), float(wz)

    def object_names(self):
        return list(self.object_paths)

    def object_pose(self, name):
        try:
            T = self._object_T(name)
            self._obj_cache[name] = T
        except Exception as exc:
            self._cb_error('object pose', exc)
            T = self._obj_cache.get(name, np.eye(4))
        return geo.pose_to_pq(T)

    def set_object_pose(self, name, position, quat_xyzw):
        self.detach(name)
        obj = self.objects[name]
        obj.set_world_poses(positions=np.array([position], dtype=float),
                            orientations=np.array([geo.xyzw_to_wxyz(quat_xyzw)]))
        obj.set_velocities(linear_velocities=np.zeros((1, 3)), angular_velocities=np.zeros((1, 3)))

    def attach(self, name, link):
        if name not in self.objects or link not in self.link_paths:
            self.log('attach: unknown object / link %s / %s' % (name, link))
            return False
        T_lo = np.linalg.inv(self._link_pose(link)) @ self._object_T(name)
        if self.attach_mode == 'joint':
            path = '%s/%s' % (GRASP_JOINTS, name)
            self._usd_scene.fixed_joint(self.stage, path, self.link_paths[link], self.object_paths[name], T_lo)
            self.grasp_joints[name] = path
        else:
            self.objects[name].set_enabled_gravities(False)
        self.held[name] = (link, T_lo)
        return True

    def detach(self, name):
        if self.held.pop(name, None) is None:
            return False
        path = self.grasp_joints.pop(name, None)
        if path:
            self.stage.RemovePrim(path)
        else:
            self.objects[name].set_enabled_gravities(True)
        self.objects[name].set_velocities(linear_velocities=np.zeros((1, 3)), angular_velocities=np.zeros((1, 3)))
        return True

    def camera_specs(self):
        return {n: s for n, s in self.cam_specs.items() if n in self.cams}

    def _set_updates(self, rp, enabled):
        try:
            rp.hydra_texture.set_updates_enabled(bool(enabled))
        except Exception as exc:
            self._cb_error('render product updates', exc)

    def set_camera_active(self, name, active):
        cam = self.cams.get(name)
        if cam and 'rp' in cam:
            self._set_updates(cam['rp'], active)

    def camera_image(self, name):
        cam = self.cams.get(name)
        if not cam or 'annot' not in cam:
            return None
        # the annotator returns a view into a buffer reused by the next frame -> copy
        data = cam['annot'].get_data(do_array_copy=True)
        if isinstance(data, dict):
            data = data.get('data')
        if data is None:
            return None
        arr = self._np(data)
        if arr.size == 0:
            return None
        if arr.ndim == 1:      # flat (H*W*4,) buffer on some builds
            w, h = self.cam_specs[name][:2]
            arr = arr.reshape(h, w, -1)
        if arr.ndim != 3 or arr.shape[2] < 3:
            return None
        return np.ascontiguousarray(arr[:, :, :3], dtype=np.uint8)
