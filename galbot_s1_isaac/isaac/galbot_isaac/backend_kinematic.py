"""Kinematic stand-in for Isaac Sim (no GPU, no physics).

Used by CI and for developing the ROS / web side on machines without Isaac Sim:
joints follow their targets with the URDF velocity limits, the base follows the
commanded velocity, grasped objects follow the gripper link and released objects
drop onto the table / tray / floor below them. Cameras produce small synthetic
images. Everything else (topics, commander, web UI) is exactly the Isaac setup.
"""
import math

import numpy as np

from . import geometry as geo
from .config import PLANAR_JOINTS
from .sim_core import settle_pose


class KinematicBackend(object):
    def __init__(self, settings, scene, urdf_xml, rc, kinematics, dt=1.0 / 60.0, log=print):
        self.settings = settings
        self.scene = scene
        self.rc = rc
        self.model = kinematics.KinematicModel(urdf_xml)
        self.dt = dt
        self.log = log
        self.t = 0.0
        self.names = [n for n, j in self.model.joints.items() if j.movable and n not in PLANAR_JOINTS]
        self.velocity_joints = set(rc.WHEEL_JOINTS)
        self.q = {n: 0.0 for n in self.names}
        self.qd = {n: 0.0 for n in self.names}
        self.q.update({n: v for n, v in rc.INITIAL_POSITIONS.items() if n in self.q})
        self.targets = {n: (self.q[n], 0.0) for n in self.names}
        self.wheel_cmd = {}
        x, y, yaw = scene.robot_spawn
        self.base = [x, y, yaw]
        self.base_vel = [0.0, 0.0, 0.0]
        self.base_cmd = (0.0, 0.0, 0.0)
        self.objects = {}
        for o in scene.objects:
            self.objects[o.name] = geo.make_pose(
                (o.position[0], o.position[1], scene.object_spawn_z(o)), geo.rpy_to_matrix(0, 0, o.yaw))
        self.held = {}   # object -> (link, T_link_obj)
        cams = settings['cameras']
        self.cams = {}
        if cams.get('enabled', True):
            for name, c in cams.get('robot', {}).items():
                self.cams[name] = (160, 120, float(c['hfov']), float(c['rate']))
            ov = scene_overview(scene)
            if ov:
                self.cams['overview_camera'] = (160, 120, ov['hfov'], float(cams.get('overview_rate', 10)))

    # ------------------------------------------------------------------ lifecycle
    def start(self):
        self.log('kinematic backend: %d joints, %d objects, cameras %s'
                 % (len(self.names), len(self.objects), sorted(self.cams)))

    def is_running(self):
        return True

    def close(self):
        pass

    def sim_time(self):
        return self.t

    # ------------------------------------------------------------------ robot
    def joint_names(self):
        return list(self.names)

    def joint_limits(self):
        return {n: (self.model.joints[n].lower, self.model.joints[n].upper, self.model.joints[n].velocity)
                for n in self.names}

    def initial_positions(self):
        return {n: self.q[n] for n in self.names if n not in self.velocity_joints}

    def set_joint_targets(self, targets):
        for n, pv in targets.items():
            if n in self.targets:
                self.targets[n] = pv

    def set_wheel_velocities(self, velocities):
        self.wheel_cmd.update(velocities)

    def set_base_command(self, vx, vy, wz):
        self.base_cmd = (vx, vy, wz)

    def reset_base(self, x, y, yaw):
        self.base = [x, y, yaw]
        self.base_vel = [0.0, 0.0, 0.0]

    def joint_state(self):
        return self.names, [self.q[n] for n in self.names], [self.qd[n] for n in self.names]

    def base_state(self):
        return tuple(self.base) + tuple(self.base_vel)

    def link_pose(self, link):
        return geo.planar_pose(*self.base) @ self.model.fk(link, self.q, base='base_footprint')

    # ------------------------------------------------------------------ objects
    def object_names(self):
        return list(self.objects)

    def object_pose(self, name):
        return geo.pose_to_pq(self.objects[name])

    def set_object_pose(self, name, position, quat_xyzw):
        self.held.pop(name, None)
        self.objects[name] = geo.pose_from_pq(position, quat_xyzw)

    def attach(self, name, link):
        if name not in self.objects or link not in self.model.links:
            self.log('attach: unknown object/link %s / %s' % (name, link))
            return False
        T_wl = self.link_pose(link)
        self.held[name] = (link, np.linalg.inv(T_wl) @ self.objects[name])
        return True

    def detach(self, name):
        if self.held.pop(name, None) is None:
            return False
        obj = self.scene.by_name[name]
        p, R = settle_pose(self.scene, obj, self.objects[name][:3, 3], self.objects[name][:3, :3])
        self.objects[name] = geo.make_pose(p, R)
        return True

    # ------------------------------------------------------------------ cameras
    def camera_specs(self):
        return dict(self.cams)

    def set_camera_active(self, name, active):
        pass

    def camera_image(self, name):
        w, h = self.cams[name][:2]
        img = np.empty((h, w, 3), dtype=np.uint8)
        img[:] = (70, 80, 95)
        # a moving bar so consecutive frames differ
        x = int((self.t * 40.0) % w)
        img[:, max(0, x - 3):x + 3] = (230, 200, 60)
        img[h // 2 - 2:h // 2 + 2, :] = (200, 200, 200)
        return img

    # ------------------------------------------------------------------ step
    def step(self):
        dt = self.dt
        for n in self.names:
            if n in self.velocity_joints:
                v = float(self.wheel_cmd.get(n, 0.0))
                self.q[n] += v * dt
                self.qd[n] = v
                continue
            target, _ff = self.targets[n]
            vmax = min(self.model.joints[n].velocity, 3.0)
            step = max(-vmax * dt, min(vmax * dt, target - self.q[n]))
            self.q[n] += step
            self.qd[n] = step / dt
        vx, vy, wz = self.base_cmd
        x, y, yaw = self.base
        c, s = math.cos(yaw), math.sin(yaw)
        vxw, vyw = c * vx - s * vy, s * vx + c * vy
        self.base = [x + vxw * dt, y + vyw * dt, geo.wrap_angle(yaw + wz * dt)]
        self.base_vel = [vxw, vyw, wz]
        for name, (link, T_lo) in self.held.items():
            self.objects[name] = self.link_pose(link) @ T_lo
        self.t += dt
        return self.t


def scene_overview(scene):
    """overview_camera entry of scene.yaml (Scene keeps the raw dict in .raw)."""
    raw = getattr(scene, 'raw', None) or {}
    cam = raw.get('overview_camera')
    if not cam:
        return None
    return {'pose': [float(v) for v in cam['pose']], 'width': int(cam.get('width', 640)),
            'height': int(cam.get('height', 480)), 'hfov': float(cam.get('hfov', 1.1))}
