"""Simulation loop shared by the Isaac Sim backend and the kinematic test backend.

The loop turns the ROS commands (same topics as the Gazebo version) into backend calls,
steps the backend and publishes the state. Backends implement:

    start()                                   build the world, put the robot in its initial pose
    step() -> sim_time                        advance one frame (physics + rendering)
    is_running() -> bool
    sim_time() -> float
    joint_names() -> [str]                    robot joints (without the planar base joints)
    joint_limits() -> {joint: (lower, upper, max_velocity)}
    initial_positions() -> {joint: value}
    set_joint_targets({joint: (position, velocity)})      position-driven joints
    set_wheel_velocities({joint: velocity})               velocity-driven joints
    set_base_command(vx, vy, wz)              body-frame velocity, integrated per physics step
    reset_base(x, y, yaw)
    joint_state() -> (names, positions, velocities)
    base_state() -> (x, y, yaw, vx_world, vy_world, wz)
    object_names() -> [str]
    object_pose(name) -> (position, quat_xyzw)
    set_object_pose(name, position, quat_xyzw)
    attach(object, link) -> bool / detach(object) -> bool
    camera_specs() -> {name: (width, height, hfov, rate)}
    set_camera_active(name, active) / camera_image(name) -> HxWx3 uint8 or None
    close()
"""
import math
import time

import numpy as np

from . import geometry as geo

FEEDFORWARD_TIMEOUT = 0.12   # s (sim) without a new command -> zero velocity feed-forward
BASE_CMD_TIMEOUT = 0.5       # s (sim) without /galbot/base/cmd_vel -> stop


def controller_table(rc):
    """controller name -> (joint names, 'position' | 'velocity')."""
    table = {ctrl: (list(joints), 'position') for ctrl, joints in rc.GROUPS.values()}
    table['steering_controller'] = (list(rc.STEERING_JOINTS), 'position')
    table['wheel_controller'] = (list(rc.WHEEL_JOINTS), 'velocity')
    return table


class SimCore(object):
    def __init__(self, backend, ros, settings, robot_model_name, rc, log=None):
        self.backend = backend
        self.ros = ros
        self.settings = settings
        self.robot_name = robot_model_name
        self.controllers = controller_table(rc)
        self.log = log or ros.info
        self.rtf = float(settings.get('realtime_factor', 1.0))
        pub = settings['publish']
        self.periods = {'joints': 1.0 / float(pub['joint_states_rate']),
                        'odom': 1.0 / float(pub['odom_rate']),
                        'objects': 1.0 / float(pub['objects_rate'])}
        self.last_pub = {k: -1e9 for k in self.periods}
        self.cameras = backend.camera_specs()
        self.last_cam = {name: -1e9 for name in self.cameras}
        self.cam_active = {}

        self.limits = backend.joint_limits()
        self.targets = {}        # joint -> position target
        self.target_time = {}    # joint -> sim time of the last command
        self.ff = {}             # joint -> velocity feed-forward
        self.wheel = {}
        for j, v in backend.initial_positions().items():
            self.targets[j] = v
            self.ff[j] = 0.0
        self.seen_seq = {}
        self.base_seq = 0
        self.base_cmd = (0.0, 0.0, 0.0)
        self.base_cmd_time = -1e9
        self.attached = {}
        self.bad_len_warned = set()
        self.frames = 0
        self.stats_wall = time.time()
        self.stats_sim = 0.0
        self.stats_frames = 0

    # ------------------------------------------------------------------ commands
    def _apply_commands(self, t):
        cmds, seqs, base_cmd, base_seq, requests = self.ros.snapshot()
        for ctrl, values in cmds.items():
            if seqs.get(ctrl) == self.seen_seq.get(ctrl):
                continue
            self.seen_seq[ctrl] = seqs.get(ctrl)
            entry = self.controllers.get(ctrl)
            if entry is None:
                continue
            joints, kind = entry
            if len(values) != len(joints):
                if ctrl not in self.bad_len_warned:
                    self.bad_len_warned.add(ctrl)
                    self.ros.warn('%s: expected %d values, got %d' % (ctrl, len(joints), len(values)))
                continue
            if kind == 'velocity':
                for j, v in zip(joints, values):
                    self.wheel[j] = float(v)
                continue
            for j, v in zip(joints, values):
                lo, hi, vmax = self.limits.get(j, (-1e9, 1e9, 1e9))
                v = min(max(float(v), lo), hi)
                old, told = self.targets.get(j), self.target_time.get(j)
                ff = 0.0
                if old is not None and told is not None and 1e-4 < t - told < 0.25:
                    ff = max(-vmax, min(vmax, (v - old) / (t - told)))
                self.targets[j], self.target_time[j], self.ff[j] = v, t, ff
        for j, tt in self.target_time.items():
            if t - tt > FEEDFORWARD_TIMEOUT:
                self.ff[j] = 0.0

        if base_seq != self.base_seq and base_cmd is not None:
            self.base_seq = base_seq
            self.base_cmd = base_cmd
            self.base_cmd_time = t
        cmd = self.base_cmd if t - self.base_cmd_time < BASE_CMD_TIMEOUT else (0.0, 0.0, 0.0)
        self.backend.set_base_command(*cmd)
        self.backend.set_joint_targets({j: (p, self.ff.get(j, 0.0)) for j, p in self.targets.items()})
        if self.wheel:
            self.backend.set_wheel_velocities(dict(self.wheel))

        changed = False
        for kind, text in requests:
            try:
                changed |= self._handle_request(kind, text)
            except Exception as exc:
                self.ros.warn('%s "%s": %s' % (kind, text, exc))
        if changed:
            self.ros.publish_attached(self.attached)

    def _handle_request(self, kind, text):
        parts = text.split()
        if kind == 'attach':
            if len(parts) < 3:
                raise ValueError('expected "<model> <link> <object> [object_link]"')
            model, link, obj = parts[:3]
            if model != self.robot_name:
                raise ValueError('unknown model %s' % model)
            if obj in self.attached:
                self.ros.warn('%s is already attached' % obj)
                return False
            if self.backend.attach(obj, link):
                self.attached[obj] = link
                self.log('attached %s to %s::%s' % (obj, model, link))
                return True
            return False
        if kind == 'detach':
            target = parts[0] if parts else '*'
            names = list(self.attached) if target in ('*', '') else [target]
            changed = False
            for name in names:
                if name in self.attached:
                    self.backend.detach(name)
                    del self.attached[name]
                    self.log('detached %s' % name)
                    changed = True
            return changed
        if kind == 'set_model_pose':
            if len(parts) < 7:
                raise ValueError('expected "<model> x y z roll pitch yaw"')
            name = parts[0]
            x, y, z, roll, pitch, yaw = [float(v) for v in parts[1:7]]
            if name == self.robot_name:
                self.backend.reset_base(x, y, yaw)
                return False
            if name not in self.backend.object_names():
                raise ValueError('unknown model %s' % name)
            changed = False
            if name in self.attached:
                self.backend.detach(name)
                del self.attached[name]
                changed = True
            q = geo.matrix_to_quat(geo.rpy_to_matrix(roll, pitch, yaw))
            self.backend.set_object_pose(name, (x, y, z), q)
            return changed
        return False

    # ------------------------------------------------------------------ publishing
    def _due(self, key, t):
        if t - self.last_pub[key] >= self.periods[key] - 1e-6:
            self.last_pub[key] = t
            return True
        return False

    def _publish(self, t):
        self.ros.publish_clock(t)
        if self._due('joints', t):
            names, pos, vel = self.backend.joint_state()
            self.ros.publish_joints(t, names, pos, vel)
        if self._due('odom', t):
            x, y, yaw, vxw, vyw, wz = self.backend.base_state()
            c, s = math.cos(yaw), math.sin(yaw)
            lin_body = (c * vxw + s * vyw, -s * vxw + c * vyw, 0.0)
            self.ros.publish_base(t, (x, y, 0.0), geo.yaw_quat(yaw), lin_body, (0.0, 0.0, wz))
        if self._due('objects', t):
            for name in self.backend.object_names():
                p, q = self.backend.object_pose(name)
                self.ros.publish_object(t, name, p, q)
        for name, (_w, _h, _hfov, rate) in self.cameras.items():
            wanted = self.ros.camera_wanted(name)
            if self.cam_active.get(name) != wanted:
                self.cam_active[name] = wanted
                self.backend.set_camera_active(name, wanted)
            if not wanted or t - self.last_cam[name] < 1.0 / rate - 1e-6:
                continue
            img = self.backend.camera_image(name)
            if img is not None:
                self.last_cam[name] = t
                self.ros.submit_image(t, name, img)

    # ------------------------------------------------------------------ loop
    def step(self):
        t = self.backend.sim_time()
        self._apply_commands(t)
        t = self.backend.step()
        self._publish(t)
        self.frames += 1
        return t

    def _stats(self, t):
        now = time.time()
        if now - self.stats_wall < 10.0:
            return
        dw = now - self.stats_wall
        rtf = (t - self.stats_sim) / dw if dw > 0 else 0.0
        fps = (self.frames - self.stats_frames) / dw if dw > 0 else 0.0
        imgs = ', '.join('%s=%d' % kv for kv in sorted(self.ros.images_published.items())) or '-'
        self.log('[stats] sim %.1fs  rtf %.2f  %.1f frames/s  images: %s  dropped %d'
                 % (t, rtf, fps, imgs, self.ros.images_dropped))
        self.stats_wall, self.stats_sim, self.stats_frames = now, t, self.frames

    def run(self, max_sim_time=None):
        t0_sim = self.backend.sim_time()
        t0_wall = time.time()
        self.stats_sim = t0_sim
        self.ros.publish_attached(self.attached)
        ros_ok = getattr(self.ros, 'ok', lambda: True)
        while self.backend.is_running() and ros_ok():
            t = self.step()
            self._stats(t)
            if max_sim_time is not None and t - t0_sim >= max_sim_time:
                break
            if self.rtf > 0:
                ahead = (t - t0_sim) / self.rtf - (time.time() - t0_wall)
                if ahead > 0:
                    time.sleep(min(ahead, 0.1))
                elif ahead < -1.0:          # slower than real time: do not try to catch up later
                    t0_sim, t0_wall = t, time.time()
        return self.frames


def joint_limits_from_model(model, joints):
    out = {}
    for j in joints:
        jt = model.joints.get(j)
        if jt is None:
            continue
        out[j] = (jt.lower, jt.upper, jt.velocity)
    return out


def support_height(scene, x, y):
    """Top surface height under (x, y) for the demo scene (used by the kinematic backend)."""
    h = 0.0
    tx, ty = scene.table_center
    dx, dy, th = scene.table_size
    if abs(x - tx) <= dx / 2.0 and abs(y - ty) <= dy / 2.0:
        h = th
        if scene.tray:
            cx, cy = scene.tray['center']
            sx, sy, sz = scene.tray['size']
            if abs(x - cx) <= sx / 2.0 and abs(y - cy) <= sy / 2.0:
                h = th + sz
    return h


def settle_pose(scene, obj, position, R):
    """Pose of a released object resting upright on the surface below it."""
    x, y = float(position[0]), float(position[1])
    z = support_height(scene, x, y) + obj.height / 2.0
    yaw = geo.yaw_of_matrix(R)
    return np.array([x, y, z]), geo.rpy_to_matrix(0.0, 0.0, yaw)
