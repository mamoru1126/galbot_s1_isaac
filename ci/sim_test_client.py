#!/usr/bin/env python3
"""End-to-end check of the running simulation (Isaac Sim or the kinematic stand-in).

Talks to the commander exactly like the web UI does (/galbot/command,
/galbot/status, /cmd_vel) and verifies:
  1. the stack comes up (joint states, odom, object poses)
  2. driving with /cmd_vel moves the base
  3. Cartesian jog + gripper commands are executed
  4. the full demo: drive to the table, pick the red can, place it on the tray, go home
  5. a left-arm pick of the blue box and placing it back
Camera frames (overview / head) are saved during the run.

Writes <out>/result.json and exits 0 on success.

Environment
  GALBOT_TEST_STARTUP_TIMEOUT  seconds to wait for the stack (default 420; Isaac Sim's first
                               start imports the robot and compiles shaders - use ~1800)
  GALBOT_TEST_TIME_SCALE       multiplies the task timeouts (Isaac Sim slower than real time)
"""
import base64
import json
import math
import os
import sys
import threading
import time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from galbot_s1_control.ros_compat import CompatNode

STARTUP_TIMEOUT = float(os.environ.get('GALBOT_TEST_STARTUP_TIMEOUT', '420'))
TIME_SCALE = float(os.environ.get('GALBOT_TEST_TIME_SCALE', '1'))
OUT = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith('_') else '/ci_out'
os.makedirs(os.path.join(OUT, 'frames'), exist_ok=True)


class Client(object):
    def __init__(self):
        self.node = CompatNode('galbot_ci_client')
        self.status = None
        self.lock = threading.Lock()
        self.frames = {}
        self.cmd_pub = self.node.publisher('/galbot/command', String, 10)
        self.vel_pub = self.node.publisher('/cmd_vel', Twist, 10)
        self.node.subscribe('/galbot/status', String, self._on_status, 5)
        for cam in ('overview', 'head', 'right_wrist'):
            self.node.subscribe('/galbot/web/%s/compressed' % cam, CompressedImage,
                                lambda m, c=cam: self._on_frame(c, m), 2)
        self.results = []
        self.t0 = time.time()

    def _on_status(self, msg):
        with self.lock:
            self.status = json.loads(msg.data)

    def _on_frame(self, cam, msg):
        with self.lock:
            self.frames[cam] = bytes(msg.data)

    def st(self):
        with self.lock:
            return self.status

    def save_frames(self, tag):
        with self.lock:
            frames = dict(self.frames)
        for cam, data in frames.items():
            with open(os.path.join(OUT, 'frames', '%s_%s.jpg' % (tag, cam)), 'wb') as f:
                f.write(data)
        return sorted(frames)

    def send(self, cmd):
        self.cmd_pub.publish(String(data=json.dumps(cmd)))

    def wait(self, pred, timeout, period=0.5):
        end = time.time() + timeout
        while time.time() < end:
            st = self.st()
            if st is not None and pred(st):
                return True
            time.sleep(period)
        return False

    def record(self, name, ok, **info):
        info.update({'name': name, 'ok': bool(ok), 'elapsed_s': round(time.time() - self.t0, 1)})
        self.results.append(info)
        print('[ci] %-28s %s %s' % (name, 'OK ' if ok else 'FAIL', json.dumps({k: v for k, v in info.items()
                                                                            if k not in ('name', 'ok')})),
              flush=True)
        return ok

    def run_task(self, cmd, timeout):
        timeout *= TIME_SCALE
        before = (self.st() or {}).get('task', {}).get('id', 0)
        self.send(cmd)
        started = self.wait(lambda s: s['task'].get('id', 0) > before, 30)
        if not started:
            return {'state': 'not_started'}
        frame_tag = 0
        end = time.time() + timeout
        while time.time() < end:
            st = self.st()
            t = st['task']
            if t['state'] in ('done', 'failed', 'cancelled'):
                return t
            if int(time.time()) % 8 == 0:
                self.save_frames('%s_%03d' % (cmd.get('name', 'task'), frame_tag))
                frame_tag += 1
                time.sleep(1.0)
            time.sleep(0.5)
        return {'state': 'timeout', 'step': self.st()['task'].get('step')}


def main():
    c = Client()
    ok_all = True
    # ---------------------------------------------------------------- 1 bring-up
    up = c.wait(lambda s: s.get('ready') and s.get('base') is not None and len(s.get('objects', {})) >= 3
                and s['task']['state'] != 'running' and s['task'].get('name') in ('_startup', ''), STARTUP_TIMEOUT)
    st = c.st() or {}
    ok_all &= c.record('bringup', up, ready=st.get('ready'), base=st.get('base'),
                       objects=sorted(st.get('objects', {})), task=st.get('task'))
    if not up:
        return finish(c, False)
    open(os.path.join(OUT, 'ready'), 'w').write('ready\n')
    time.sleep(12.0)  # give the camera relay / host screenshot some time
    cams = c.save_frames('00_idle')
    c.record('camera_frames', True, cameras=cams)

    # ---------------------------------------------------------------- 2 drive
    x0 = c.st()['base']['x']
    cmd = Twist()
    cmd.linear.x = 0.25
    end = time.time() + 3.0
    while time.time() < end:
        c.vel_pub.publish(cmd)
        time.sleep(0.05)
    c.vel_pub.publish(Twist())
    time.sleep(2.0)
    dx = c.st()['base']['x'] - x0
    ok_all &= c.record('drive_forward', 0.05 < dx < 1.5, dx=round(dx, 3))

    # ---------------------------------------------------------------- 3 jog + gripper
    j_before = dict(c.st()['setpoint'])
    for _ in range(3):
        c.send({'type': 'jog', 'arm': 'right', 'dz': 0.02})
        time.sleep(0.6)
    time.sleep(1.5)
    j_after = c.st()['setpoint']
    moved = max(abs(j_after[k] - j_before[k]) for k in j_after if k.startswith('right_arm_joint'))
    ok_all &= c.record('jog_right_arm', moved > 0.01, max_joint_change=round(moved, 3))
    c.send({'type': 'gripper', 'arm': 'right', 'action': 'close'})
    closed = c.wait(lambda s: s['gripper']['right']['closed_fraction'] > 0.8, 20 * TIME_SCALE)
    c.send({'type': 'gripper', 'arm': 'right', 'action': 'open'})
    opened = c.wait(lambda s: s['gripper']['right']['closed_fraction'] < 0.1, 20 * TIME_SCALE)
    ok_all &= c.record('gripper_close_open', closed and opened)
    c.send({'type': 'pose', 'name': 'ready'})
    time.sleep(3.0)

    # ---------------------------------------------------------------- 4 full demo
    t = c.run_task({'type': 'task', 'name': 'demo', 'object': 'red_can'}, 900)
    st = c.st()
    can = st['objects'].get('red_can', {})
    tray = st['scene']['tray']
    on_tray = (abs(can.get('x', 99) - tray['center'][0]) < tray['size'][0] / 2 and
               abs(can.get('y', 99) - tray['center'][1]) < tray['size'][1] / 2 and
               abs(can.get('z', 0) - (st['scene']['table']['size'][2] + tray['size'][2] + 0.06)) < 0.03)
    home = st['base'] and math.hypot(st['base']['x'], st['base']['y']) < 0.1
    undisturbed = untouched(st, ('green_bottle', 'blue_box'))
    ok_all &= c.record('demo_pick_place_red_can', t.get('state') == 'done' and on_tray and home,
                       task=t, red_can=can, base=st['base'], log=st.get('log', [])[-8:])
    ok_all &= c.record('other_objects_undisturbed', not undisturbed, moved=undisturbed)
    c.save_frames('90_after_demo')

    # ---------------------------------------------------------------- 5 left arm
    t = c.run_task({'type': 'task', 'name': 'pick', 'object': 'blue_box', 'arm': 'auto'}, 600)
    st = c.st()
    box = st['objects'].get('blue_box', {})
    lifted = box.get('z', 0) > st['scene']['table']['size'][2] + 0.05 + 0.08
    ok_all &= c.record('pick_blue_box_left_arm', t.get('state') == 'done' and lifted and
                       st['holding'].get('left') == 'blue_box', task=t, blue_box=box, holding=st['holding'])
    c.save_frames('95_holding_box')
    t = c.run_task({'type': 'task', 'name': 'place', 'target': 'origin'}, 600)
    st = c.st()
    box = st['objects'].get('blue_box', {})
    back = abs(box.get('x', 9) - 1.45) < 0.03 and abs(box.get('y', 9) - 0.35) < 0.03 and \
        abs(box.get('z', 0) - (st['scene']['table']['size'][2] + 0.05)) < 0.03
    ok_all &= c.record('place_blue_box_origin', t.get('state') == 'done' and back, task=t, blue_box=box)
    moved = untouched(st, ('green_bottle',))
    can = st['objects'].get('red_can', {})
    still_on_tray = abs(can.get('x', 9) - tray['center'][0]) < 0.08 and abs(can.get('y', 9) - tray['center'][1]) < 0.08
    ok_all &= c.record('scene_intact_after_left_arm', not moved and still_on_tray, moved=moved, red_can=can)

    t = c.run_task({'type': 'reset_scene'}, 300)
    ok_all &= c.record('reset_scene', t.get('state') == 'done', task=t)
    return finish(c, ok_all)


def untouched(st, names):
    """Names of objects that moved away from their spawn pose (should be empty)."""
    moved = []
    spawn = {o['name']: o for o in st['scene']['objects']}
    for name in names:
        p = st['objects'].get(name)
        o = spawn.get(name)
        if p is None or o is None:
            moved.append(name)
            continue
        z0 = st['scene']['table']['size'][2] + o['size'][2] / 2.0
        if 'position' in o:
            x0, y0 = o['position']
            if math.hypot(p['x'] - x0, p['y'] - y0) > 0.02:
                moved.append(name)
                continue
        if abs(p['z'] - z0) > 0.015:
            moved.append(name)
    return moved


def finish(c, ok):
    st = c.st() or {}
    with open(os.path.join(OUT, 'result.json'), 'w') as f:
        json.dump({'ok': bool(ok), 'results': c.results, 'final_log': st.get('log', [])}, f,
                  indent=2, ensure_ascii=False)
    print('[ci] RESULT %s' % ('PASS' if ok else 'FAIL'), flush=True)
    time.sleep(8.0)  # final screenshot window
    c.node.shutdown()
    os._exit(0 if ok else 1)


if __name__ == '__main__':
    main()
