#!/usr/bin/env python3
"""Galbot S1 commander (ROS 1 / ROS 2).

* receives JSON commands on /galbot/command (std_msgs/String) - sent by the web UI
* streams smooth joint setpoints to the forward position controllers
* Cartesian jogging / IK for both arms, gripper with grasp-attach in simulation
* demo tasks: drive to the table, pick an object, place it on the tray, go home
* publishes a JSON status on /galbot/status for the web UI

Command examples (see README for the full list):
  {"type": "pose", "name": "ready"}
  {"type": "joint", "name": "right_arm_joint4", "position": 1.2}
  {"type": "jog", "arm": "right", "dx": 0.02, "dy": 0, "dz": 0, "dyaw": 0}
  {"type": "gripper", "arm": "right", "action": "close"}
  {"type": "task", "name": "pick", "object": "red_can", "arm": "auto"}
  {"type": "task", "name": "demo"}
  {"type": "stop"}
"""
import json
import math
import queue
import threading
import time
import traceback

import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from galbot_s1_control import robot_config as rc
from galbot_s1_control.grasp import GraspPlanner
from galbot_s1_control.kinematics import (KinematicModel, planar_pose_matrix, pose_to_matrix,
                                          rpy_to_matrix, wrap_angle, yaw_of)
from galbot_s1_control.motion import JointPath
from galbot_s1_control.ros_compat import CompatNode, forward_command_topic
from galbot_s1_control.scene import Scene


class TaskCancelled(Exception):
    pass


class TaskError(Exception):
    pass


SIDES = ('left', 'right')


class Commander(object):
    def __init__(self):
        n = self.node = CompatNode('galbot_commander')
        urdf = n.param('robot_description', '')
        wait_start = time.time()
        while not urdf and n.ok() and time.time() - wait_start < 30.0:
            time.sleep(0.5)
            urdf = n.param('robot_description', '')
        if not urdf:
            raise RuntimeError('robot_description parameter is empty')
        self.model = KinematicModel(urdf)
        self.planner = GraspPlanner(self.model)
        self.scene = Scene.from_file(n.param('scene_file', ''))
        self.robot_name = n.param('robot_model_name', self.scene.robot_name)
        self.rate = float(n.param('control_rate', 50.0))
        self.vel = {
            'arm': float(n.param('arm_max_velocity', 0.8)),
            'gripper': float(n.param('gripper_max_velocity', 2.0)),
            'torso': float(n.param('torso_max_velocity', 0.12)),
            'head': float(n.param('head_max_velocity', 1.0)),
        }
        self.drive_speed = float(n.param('drive_max_speed', 0.35))
        self.settle_tol = float(n.param('settle_tolerance', 0.02))
        self.startup_pose = n.param('startup_pose', 'ready')
        odom_topic = n.param('ground_truth_topic', '/galbot/ground_truth/odom')

        self.lock = threading.RLock()
        self.joints = {}
        self.setpoint = {}
        self.paths = {}
        self.last_publish = {}
        self.initialized = False
        self.base = None
        self.base_stamp = 0.0
        self.objects = {}
        self.holding = {s: None for s in SIDES}
        self.hold_info = {s: None for s in SIDES}
        self.log_lines = []
        self.task = {'name': '', 'state': 'idle', 'step': '', 'error': '', 'id': 0}
        self.cancel = threading.Event()
        self.tasks = queue.Queue()
        self.drive_active = False

        # ---------------------------------------------------------------- pubs
        self.group_pubs = {g: n.publisher(forward_command_topic(ctrl), Float64MultiArray, 10)
                           for g, (ctrl, _) in rc.GROUPS.items()}
        self.cmd_vel_pub = n.publisher('/cmd_vel', Twist, 10)
        self.attach_pub = n.publisher('/galbot/sim/attach', String, 10)
        self.detach_pub = n.publisher('/galbot/sim/detach', String, 10)
        self.set_pose_pub = n.publisher('/galbot/sim/set_model_pose', String, 10)
        self.status_pub = n.publisher('/galbot/status', String, 10)

        # ---------------------------------------------------------------- subs
        n.subscribe('/joint_states', JointState, self._on_joint_states, 50)
        n.subscribe(odom_topic, Odometry, self._on_base_odom, 10)
        for obj in self.scene.objects:
            n.subscribe('/galbot/objects/%s/odom' % obj.name, Odometry,
                        lambda msg, name=obj.name: self._on_object_odom(name, msg), 10)
        n.subscribe('/galbot/command', String, self._on_command, 50)

        n.thread_timer(1.0 / self.rate, self._control_step, 'control')
        n.thread_timer(0.2, self._publish_status, 'status')
        self.worker = threading.Thread(target=self._worker, daemon=True)
        self.worker.start()
        self.log('commander started (ROS%d)' % (2 if n.raw is not None else 1))

    # ================================================================== callbacks
    def _on_joint_states(self, msg):
        with self.lock:
            for name, pos in zip(msg.name, msg.position):
                self.joints[name] = float(pos)
            if not self.initialized:
                needed = [j for _, (_, js) in rc.GROUPS.items() for j in js]
                if all(j in self.joints for j in needed):
                    for j in needed:
                        self.setpoint[j] = self.joints[j]
                    self.initialized = True
                    self.log('joint states received - ready')
                    if self.startup_pose and self.startup_pose in rc.POSES:
                        self.tasks.put(('_startup', {}))

    def _on_base_odom(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z))
        with self.lock:
            self.base = (p.x, p.y, yaw)
            self.base_stamp = time.time()

    def _on_object_odom(self, name, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        T = pose_to_matrix((p.x, p.y, p.z), (o.x, o.y, o.z, o.w))
        with self.lock:
            self.objects[name] = T

    def _on_command(self, msg):
        try:
            cmd = json.loads(msg.data)
        except ValueError:
            self.log('invalid command json: %s' % msg.data[:80], level='warn')
            return
        try:
            self.handle_command(cmd)
        except Exception as exc:  # never kill the subscriber thread
            self.log('command %s failed: %s' % (cmd.get('type'), exc), level='error')
            traceback.print_exc()

    # ================================================================== logging / status
    def log(self, text, level='info'):
        stamp = time.strftime('%H:%M:%S')
        with self.lock:
            self.log_lines.append('%s %s' % (stamp, text))
            self.log_lines = self.log_lines[-40:]
        getattr(self.node, {'info': 'info', 'warn': 'warn', 'error': 'error'}[level])(text)

    def _publish_status(self):
        with self.lock:
            st = {
                'stamp': self.node.now(),
                'ready': self.initialized,
                'task': dict(self.task),
                'log': list(self.log_lines[-12:]),
                'holding': dict(self.holding),
                'base': None if self.base is None else
                {'x': self.base[0], 'y': self.base[1], 'yaw': self.base[2]},
                'objects': {k: {'x': T[0, 3], 'y': T[1, 3], 'z': T[2, 3], 'yaw': yaw_of(T)}
                            for k, T in self.objects.items()},
                'joints': {k: round(v, 4) for k, v in self.joints.items()
                           if not k.startswith('wheel_')},
                'setpoint': {k: round(v, 4) for k, v in self.setpoint.items()},
                'gripper': {},
                'scene': self.scene.to_dict(),
                'limits': self._limits_cache(),
            }
            for s in SIDES:
                g = self.planner.gripper[s]
                a = self.joints.get(g.j1, 0.0)
                st['gripper'][s] = {'angle': a, 'opening': g.opening_for_angle(a),
                                    'closed_fraction': (a - g.lower) / (g.upper - g.lower)}
        self.status_pub.publish(String(data=json.dumps(st)))

    def _limits_cache(self):
        if not hasattr(self, '_limits'):
            self._limits = {}
            for _, (_, names) in rc.GROUPS.items():
                for nme in names:
                    lo, hi = self.model.limits(nme)
                    self._limits[nme] = [lo, hi]
        return self._limits

    # ================================================================== control loop
    def _control_step(self):
        t = self.node.now()
        with self.lock:
            if not self.initialized:
                return
            for group, (_, names) in rc.GROUPS.items():
                path = self.paths.get(group)
                if path is not None:
                    pos = path.sample(t)
                    for nme, v in zip(names, pos):
                        self.setpoint[nme] = float(v)
                    if path.finished(t):
                        del self.paths[group]
                    self._publish_group(group)
                elif time.time() - self.last_publish.get(group, 0.0) > 1.0:
                    self._publish_group(group)

    def _publish_group(self, group):
        names = rc.GROUPS[group][1]
        self.group_pubs[group].publish(Float64MultiArray(data=[self.setpoint[nme] for nme in names]))
        self.last_publish[group] = time.time()

    # ================================================================== motion primitives
    def _vel_for(self, group):
        if group.endswith('_arm'):
            return self.vel['arm']
        if group.endswith('_gripper'):
            return self.vel['gripper']
        return self.vel[group]

    def move_group(self, group, target, waypoints=None, duration=None, speed=1.0, wait=False,
                   timeout=None):
        names = rc.GROUPS[group][1]
        target = [self.model.clamp(nme, float(v)) for nme, v in zip(names, target)]
        pts = [list(w) for w in (waypoints or [])] + [target]
        with self.lock:
            start = [self.setpoint[nme] for nme in names]
            path = JointPath(start, pts, self._vel_for(group) * speed, self.node.now(),
                             min_duration=0.25, duration=duration)
            self.paths[group] = path
        if wait:
            self.wait_path(group, path, timeout)
        return path

    def wait_path(self, group, path, timeout=None):
        names = rc.GROUPS[group][1]
        limit = (timeout if timeout is not None else path.duration + 6.0)

        def done():
            self.check_cancel()
            with self.lock:
                current = self.paths.get(group)
                if current is not None and current is not path:
                    return True  # superseded by a newer command
                if self.node.now() - path.t0 < path.duration:
                    return False
                err = max(abs(self.joints.get(nme, 0.0) - g) for nme, g in zip(names, path.goal))
            return err < self.settle_tol

        if not self.node.wait_for(done, limit):
            with self.lock:
                if self.paths.get(group) not in (None, path):
                    return
                err = max(abs(self.joints.get(nme, 0.0) - g) for nme, g in zip(names, path.goal))
            self.log('%s: target not reached (err %.3f) - continuing' % (group, err), level='warn')

    def move_arm(self, side, target, **kw):
        return self.move_group('%s_arm' % side, target, **kw)

    def move_gripper(self, side, angle, **kw):
        g = self.planner.gripper[side]
        q = g.joint_positions(angle)
        return self.move_group('%s_gripper' % side, [q[nme] for nme in g.joint_names()], **kw)

    def go_pose(self, name, wait=False, groups=None):
        pose = rc.POSES[name]
        paths = []
        for group, vals in pose.items():
            if groups and group not in groups:
                continue
            if group.endswith('_arm') and self.holding[group[:-4]] and name != 'ready':
                continue
            paths.append((group, self.move_group(group, vals)))
        if wait:
            for group, p in paths:
                self.wait_path(group, p)

    def stop_all(self):
        with self.lock:
            self.paths.clear()
            # hold the currently measured position
            for nme in list(self.setpoint):
                if nme in self.joints:
                    self.setpoint[nme] = self.joints[nme]
            for g in rc.GROUPS:
                self._publish_group(g)
        self.cmd_vel_pub.publish(Twist())

    # ================================================================== state helpers
    def q_setpoint(self):
        with self.lock:
            return dict(self.setpoint)

    def base_matrix(self):
        with self.lock:
            if self.base is None:
                raise TaskError('ロボットの位置 (odom) が取得できません')
            return planar_pose_matrix(*self.base)

    def object_matrix(self, name):
        with self.lock:
            T = self.objects.get(name)
        if T is None:
            raise TaskError('物体 %s の位置が取得できません' % name)
        return T.copy()

    def check_cancel(self):
        if self.cancel.is_set():
            raise TaskCancelled()

    # ================================================================== gripper / grasp
    def grasp_candidate(self, side, only=None):
        """Object currently between the jaws of `side` (or None).
        Returns (name, close_angle, width)."""
        with self.lock:
            if self.base is None:
                return None
            q = dict(self.joints)
            objects = dict(self.objects)
        g = self.planner.gripper[side]
        T_wg = planar_pose_matrix(*self.base) @ self.model.fk(g.base, q, rc.ROBOT_BASE_FRAME)
        T_gw = np.linalg.inv(T_wg)
        pa_open = g.active_contact(q.get(g.j1, g.lower))
        pad_y = 0.5 * (g.fixed_contact[1] + pa_open[1])
        closing_yaw = math.atan2(T_wg[1, 2], T_wg[0, 2])
        best = None
        for name, T_wo in objects.items():
            if only and name != only:
                continue
            if name in self.holding.values():
                continue
            obj = self.scene.by_name.get(name)
            if obj is None:
                continue
            p = T_gw @ T_wo[:, 3]
            width = obj.width_along(closing_yaw, yaw_of(T_wo))
            # object axis must be roughly along the approach axis (top-down grasp)
            if abs(T_gw[:3, :3] @ T_wo[:3, 2] @ np.array([1.0, 0, 0])) < 0.7:
                continue
            top_x = p[0] - obj.height / 2.0
            bottom_x = p[0] + obj.height / 2.0
            if not (top_x < 0.12 and bottom_x > 0.095):
                continue
            if abs(p[1] - pad_y) > 0.035:
                continue
            near = p[2] - width / 2.0
            far = p[2] + width / 2.0
            if near < g.fixed_contact[2] - 0.012 or far > pa_open[2] + 0.012:
                continue
            opening = far + 0.003 - g.fixed_contact[2]
            angle = g.angle_for_opening(opening)
            score = abs(p[1] - pad_y) + abs(near - g.fixed_contact[2])
            if best is None or score < best[3]:
                best = (name, angle, width, score)
        return None if best is None else best[:3]

    def close_gripper(self, side, wait=True, tcp=None, expected=None):
        g = self.planner.gripper[side]
        if self.holding[side]:
            return self.holding[side]
        cand = self.grasp_candidate(side, only=expected)
        if cand is None and expected:
            self.log('%s が把持範囲にありません' % expected, level='warn')
        if cand is None:
            self.move_gripper(side, g.closed_angle, wait=wait)
            return None
        name, angle, width = cand
        self.move_gripper(side, angle, wait=wait)
        self.attach(side, name, tcp if tcp is not None else g.tcp_in_link7(width))
        return name

    def attach(self, side, name, tcp):
        self.attach_pub.publish(String(data='%s %s_arm_link7 %s' % (self.robot_name, side, name)))
        with self.lock:
            q = dict(self.joints)
            T_wb = planar_pose_matrix(*self.base)
            T_wo = self.objects[name].copy()
        T_w_tcp = T_wb @ self.planner.ik[side].fk(q, tcp=tcp)
        self.holding[side] = name
        self.hold_info[side] = {'tcp': tcp, 'T_tcp_obj': np.linalg.inv(T_w_tcp) @ T_wo}
        self.log('%s を把持しました (%s アーム)' % (self.label(name), side))

    def release(self, side, wait=True):
        name = self.holding[side]
        if name:
            self.detach_pub.publish(String(data=name))
            self.holding[side] = None
            self.hold_info[side] = None
            self.log('%s を離しました' % self.label(name))
        g = self.planner.gripper[side]
        self.move_gripper(side, g.open_angle, wait=wait)

    def label(self, name):
        obj = self.scene.by_name.get(name)
        return obj.label if obj else name

    # ================================================================== base navigation
    def drive_to(self, x, y, yaw, tol_xy=0.008, tol_yaw=0.01, timeout=60.0):
        start = self.node.now()
        period = 0.05
        self.drive_active = True
        try:
            while True:
                self.check_cancel()
                with self.lock:
                    if self.base is None:
                        raise TaskError('odom がありません')
                    bx, by, byaw = self.base
                ex, ey = x - bx, y - by
                eyaw = wrap_angle(yaw - byaw)
                c, s = math.cos(byaw), math.sin(byaw)
                exr, eyr = c * ex + s * ey, -s * ex + c * ey
                dist = math.hypot(ex, ey)
                if dist < tol_xy and abs(eyaw) < tol_yaw:
                    break
                if self.node.now() - start > timeout:
                    raise TaskError('移動がタイムアウトしました')
                cmd = Twist()
                if abs(eyaw) > 0.4 and dist > 0.05:
                    cmd.angular.z = max(-0.6, min(0.6, 1.5 * eyaw))
                else:
                    k = 1.4
                    vx, vy = k * exr, k * eyr
                    v = math.hypot(vx, vy)
                    if v > self.drive_speed:
                        vx, vy = vx * self.drive_speed / v, vy * self.drive_speed / v
                    # small minimum speed so the base does not creep forever
                    if 0 < v < 0.02 and dist > tol_xy:
                        vx, vy = vx * 0.02 / v, vy * 0.02 / v
                    cmd.linear.x, cmd.linear.y = vx, vy
                    cmd.angular.z = max(-0.6, min(0.6, 1.5 * eyaw))
                self.cmd_vel_pub.publish(cmd)
                self.node.sleep(period)
        finally:
            self.cmd_vel_pub.publish(Twist())
            self.drive_active = False
        self.node.sleep(0.3)

    def carry_clearance(self, held):
        """Height the bottom of a carried object must have to pass over the others."""
        others = [o.height for o in self.scene.objects if o.name != held]
        return self.scene.table_height + (max(others) if others else 0.0) + 0.05

    def raise_torso_for_carry(self, name):
        obj = self.scene.by_name[name]
        bottom = self.object_matrix(name)[2, 3] - obj.height / 2.0
        need = self.carry_clearance(name) - bottom
        if need <= 0.0:
            return
        with self.lock:
            torso = self.setpoint['torso_lift_joint1']
        target = self.model.clamp('torso_lift_joint1', torso + need)
        self.step('胴を %.0f cm 上げて他の物の上を通します' % ((target - torso) * 100))
        self.move_group('torso', [target], wait=True)

    def lower_torso_to_ready(self):
        ready = rc.POSES['ready']['torso'][0]
        with self.lock:
            torso = self.setpoint['torso_lift_joint1']
        if torso > ready + 1e-3:
            self.step('胴を下げる')
            self.move_group('torso', [ready], wait=True)

    def base_goal_for(self, side, target_xy):
        sx, sy = self.planner.SWEET_SPOT[side]
        gx, gy = target_xy[0] - sx, target_xy[1] - sy
        max_x = self.scene.table_front_x - 0.40
        if gx > max_x:
            gx = max_x
        return gx, gy, 0.0

    # ================================================================== tasks
    def _worker(self):
        while self.node.ok():
            try:
                name, args = self.tasks.get(timeout=0.2)
            except queue.Empty:
                continue
            self.cancel.clear()
            with self.lock:
                self.task = {'name': name, 'state': 'running', 'step': '', 'error': '',
                             'id': self.task['id'] + 1}
            try:
                getattr(self, 'task_' + name.lstrip('_'))(**args)
                with self.lock:
                    self.task['state'] = 'done'
                    self.task['step'] = ''
                if not name.startswith('_'):
                    self.log('タスク %s 完了' % name)
            except TaskCancelled:
                self.stop_all()
                with self.lock:
                    self.task['state'] = 'cancelled'
                self.log('タスク %s を停止しました' % name, level='warn')
            except Exception as exc:
                self.stop_all()
                with self.lock:
                    self.task['state'] = 'failed'
                    self.task['error'] = str(exc)
                self.log('タスク %s 失敗: %s' % (name, exc), level='error')
                if not isinstance(exc, TaskError):
                    traceback.print_exc()

    def step(self, text):
        with self.lock:
            self.task['step'] = text
        self.log(text)

    def busy(self):
        return self.task['state'] == 'running' or not self.tasks.empty()

    def task_startup(self):
        self.node.sleep(0.5)
        self.go_pose(self.startup_pose, wait=True)
        self.release('left', wait=False)
        self.release('right', wait=True)

    def choose_arm(self, obj_name, arm):
        if arm in SIDES:
            return arm
        T = self.object_matrix(obj_name)
        return 'right' if T[1, 3] <= self.scene.table_center[1] else 'left'

    def task_pick(self, object='red_can', arm='auto'):
        if object not in self.scene.by_name:
            raise TaskError('不明な物体: %s' % object)
        obj = self.scene.by_name[object]
        side = self.choose_arm(object, arm)
        if self.holding[side]:
            raise TaskError('%s アームは既に %s を持っています' % (side, self.label(self.holding[side])))
        self.step('準備姿勢へ (%s アームで %s を掴みます)' % (side, obj.label))
        self.go_pose('ready', wait=True)
        self.release(side, wait=True)
        T_wo = self.object_matrix(object)
        gx, gy, gyaw = self.base_goal_for(side, (T_wo[0, 3], T_wo[1, 3]))
        self.step('テーブルへ移動 (x=%.2f, y=%.2f)' % (gx, gy))
        self.drive_to(gx, gy, gyaw)
        T_bo = np.linalg.inv(self.base_matrix()) @ self.object_matrix(object)
        self.step('把持姿勢を計算中')
        plan = self.planner.plan_topdown_grasp(side, obj, T_bo, self.q_setpoint())
        if plan is None:
            raise TaskError('IK で把持姿勢が見つかりませんでした')
        self.step('%s の上へアームを移動' % obj.label)
        self.move_arm(side, plan['pregrasp'], wait=True)
        self.step('下降')
        self.move_arm(side, plan['descend'][-1], waypoints=plan['descend'][:-1], speed=0.4, wait=True)
        self.node.sleep(0.3)
        self.step('グリッパを閉じる')
        got = self.close_gripper(side, tcp=plan['tcp'], expected=object)
        if got != object:
            self.release(side)
            self.move_arm(side, plan['pregrasp'], wait=True)
            raise TaskError('%s を掴めませんでした' % obj.label)
        self.node.sleep(0.3)
        self.step('持ち上げ')
        self.move_arm(side, plan['lift'][-1], waypoints=plan['lift'][:-1], speed=0.5, wait=True)
        self.node.sleep(0.5)
        z_now = self.object_matrix(object)[2, 3]
        z0 = T_wo[2, 3]
        if z_now < z0 + 0.05:
            raise TaskError('持ち上げを確認できませんでした (dz=%.3f)' % (z_now - z0))
        self.log('%s を持ち上げました (dz=%.3f m)' % (obj.label, z_now - z0))
        self.raise_torso_for_carry(object)

    def task_place(self, arm='auto', target='tray', x=None, y=None):
        sides = [s for s in SIDES if self.holding[s]]
        if arm in SIDES:
            sides = [arm] if self.holding[arm] else []
        if not sides:
            raise TaskError('何も持っていません')
        side = sides[0]
        name = self.holding[side]
        obj = self.scene.by_name[name]
        info = self.hold_info[side]
        if target == 'tray' and self.scene.tray:
            tx, ty = self.scene.tray['center']
            surface = self.scene.tray_top()
            where = 'トレイ'
        elif target == 'origin':
            tx, ty = obj.position
            surface = self.scene.table_height
            where = '元の位置'
        else:
            tx, ty = float(x), float(y)
            surface = self.scene.table_height
            where = '(%.2f, %.2f)' % (tx, ty)
        gx, gy, gyaw = self.base_goal_for(side, (tx, ty))
        self.step('%s を%sへ運びます' % (obj.label, where))
        self.drive_to(gx, gy, gyaw)
        self.lower_torso_to_ready()
        T_wb = self.base_matrix()
        obj_yaw = yaw_of(self.object_matrix(name))
        T_w_obj = planar_pose_matrix(tx, ty, obj_yaw, surface + obj.height / 2.0 + 0.004)
        T_b_tcp = np.linalg.inv(T_wb) @ T_w_obj @ np.linalg.inv(info['T_tcp_obj'])
        plan = self.planner.plan_place(side, T_b_tcp, self.q_setpoint(), info['tcp'])
        if plan is None:
            raise TaskError('置く姿勢の IK が解けませんでした')
        self.step('置く位置の上へ')
        self.move_arm(side, plan['pre'], wait=True)
        self.step('下降')
        self.move_arm(side, plan['descend'][-1], waypoints=plan['descend'][:-1], speed=0.4, wait=True)
        self.node.sleep(0.3)
        self.step('リリース')
        self.release(side, wait=True)
        self.node.sleep(0.3)
        self.step('アームを上げる')
        self.move_arm(side, plan['retreat'][-1], waypoints=plan['retreat'][:-1], speed=0.5, wait=True)
        self.go_pose('ready', wait=True, groups=['%s_arm' % side])

    def task_home(self):
        self.step('準備姿勢へ')
        self.go_pose('ready', wait=True)
        x, y, yaw = self.scene.robot_spawn
        self.step('スタート位置へ戻る')
        with self.lock:
            bx = self.base[0] if self.base else x
        # back away from the table first, then go to the start pose
        if bx > x + 0.3:
            self.drive_to(bx - 0.3, self.base[1], 0.0, tol_xy=0.03, tol_yaw=0.05)
        self.drive_to(x, y, yaw, tol_xy=0.02, tol_yaw=0.03)

    def task_approach(self, object='red_can', arm='auto'):
        side = self.choose_arm(object, arm)
        T_wo = self.object_matrix(object)
        self.go_pose('ready', wait=True)
        gx, gy, gyaw = self.base_goal_for(side, (T_wo[0, 3], T_wo[1, 3]))
        self.step('%s の前へ移動' % self.label(object))
        self.drive_to(gx, gy, gyaw)

    def task_demo(self, object='red_can', arm='auto'):
        self.log('デモ開始: %s を掴んでトレイに置き、スタート位置へ戻ります' % self.label(object))
        self.task_pick(object=object, arm=arm)
        self.task_place(arm='auto', target='tray')
        self.task_home()

    def task_reset_scene(self):
        self.step('シーンをリセット')
        for s in SIDES:
            if self.holding[s]:
                self.release(s, wait=False)
        self.detach_pub.publish(String(data='*'))
        self.node.sleep(0.3)
        for obj in self.scene.objects:
            x, y = obj.position
            self.set_pose_pub.publish(String(data='%s %.4f %.4f %.4f 0 0 %.4f' % (
                obj.name, x, y, self.scene.object_spawn_z(obj) + 0.002, obj.yaw)))
        self.node.sleep(0.5)
        self.go_pose('ready', wait=True)

    # ================================================================== commands
    def handle_command(self, cmd):
        ctype = cmd.get('type')
        if ctype == 'stop':
            self.cancel.set()
            while not self.tasks.empty():
                try:
                    self.tasks.get_nowait()
                except queue.Empty:
                    break
            self.stop_all()
            self.log('停止')
            return
        if ctype in ('task', 'reset_scene'):
            if self.busy():
                self.log('他のタスクを実行中です (stop で中断できます)', level='warn')
                return
            name = 'reset_scene' if ctype == 'reset_scene' else cmd.get('name', '')
            if not hasattr(self, 'task_' + name) or name.startswith('_'):
                self.log('不明なタスク: %s' % name, level='warn')
                return
            args = {k: v for k, v in cmd.items() if k not in ('type', 'name')}
            self.tasks.put((name, args))
            return
        if not self.initialized:
            self.log('まだ joint_states を受信していません', level='warn')
            return
        if self.busy():
            self.log('タスク実行中は手動操作できません (stop で中断)', level='warn')
            return
        self.cancel.clear()  # a previous "stop" must not abort manual gripper waits
        if ctype == 'pose':
            name = cmd.get('name', 'ready')
            if name in rc.POSES:
                self.go_pose(name)
        elif ctype == 'joint':
            nme = cmd['name']
            for group, (_, names) in rc.GROUPS.items():
                if nme in names:
                    with self.lock:
                        target = [self.setpoint[j] for j in names]
                    target[names.index(nme)] = float(cmd['position'])
                    self.move_group(group, target)
                    return
        elif ctype == 'joints':
            self.move_group(cmd['group'], cmd['positions'])
        elif ctype == 'torso':
            self.move_group('torso', [float(cmd['position'])])
        elif ctype == 'head':
            with self.lock:
                pan = self.setpoint['head_joint1']
                tilt = self.setpoint['head_joint2']
            self.move_group('head', [float(cmd.get('pan', pan)), float(cmd.get('tilt', tilt))])
        elif ctype == 'jog':
            self.jog(cmd.get('arm', 'right'), [float(cmd.get('dx', 0)), float(cmd.get('dy', 0)),
                                               float(cmd.get('dz', 0))], float(cmd.get('dyaw', 0)))
        elif ctype == 'gripper':
            side = cmd.get('arm', 'right')
            action = cmd.get('action')
            if action == 'open':
                threading.Thread(target=self.release, args=(side, False), daemon=True).start()
            elif action == 'close':
                threading.Thread(target=self._safe, args=(self.close_gripper, side), daemon=True).start()
            elif action == 'toggle':
                if self.holding[side]:
                    threading.Thread(target=self.release, args=(side, False), daemon=True).start()
                else:
                    threading.Thread(target=self._safe, args=(self.close_gripper, side), daemon=True).start()
            elif 'position' in cmd:  # 0 = open, 1 = closed
                g = self.planner.gripper[side]
                frac = min(max(float(cmd['position']), 0.0), 1.0)
                self.move_gripper(side, g.lower + frac * (g.upper - g.lower))
        else:
            self.log('不明なコマンド: %s' % ctype, level='warn')

    def _safe(self, fn, *args):
        try:
            fn(*args)
        except Exception as exc:
            self.log('%s: %s' % (fn.__name__, exc), level='error')

    def jog(self, side, d, dyaw):
        names = rc.ARM_JOINTS[side]
        q = self.q_setpoint()
        info = self.hold_info[side]
        tcp = info['tcp'] if info else self.planner.gripper[side].tcp_in_link7(0.04)
        T = self.planner.ik[side].fk(q, tcp=tcp)
        T_t = T.copy()
        T_t[:3, 3] += np.asarray(d)
        if dyaw:
            T_t[:3, :3] = rpy_to_matrix(0, 0, dyaw) @ T[:3, :3]
        sol, ok, ep, er = self.planner.ik[side].solve(T_t, q, q_fixed=q, tcp=tcp, restarts=0,
                                                      max_iters=100)
        if not ok:
            self.log('IK が解けません (到達範囲外かも)', level='warn')
            return
        self.move_arm(side, [sol[nme] for nme in names], speed=0.6)


def main():
    commander = Commander()
    commander.node.spin()


if __name__ == '__main__':
    main()
