#!/usr/bin/env python3
"""Galbot S1 omni base controller (ROS 1 / ROS 2).

/cmd_vel (geometry_msgs/Twist, body frame)
   -> velocity / acceleration limits + command timeout
   -> /galbot/base/cmd_vel      consumed by the simulator (planar base motion)
   -> swerve wheel animation    /steering_controller (positions), /wheel_controller (velocities)

/galbot/ground_truth/odom (from the simulator, frame odom == world)
   -> /odom + TF odom -> base_footprint
"""
import math
import time
import threading

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

from galbot_s1_control import robot_config as rc
from galbot_s1_control.ros_compat import CompatNode, forward_command_topic
from galbot_s1_control.swerve import SwerveKinematics


def _approach(current, target, max_step):
    if target > current + max_step:
        return current + max_step
    if target < current - max_step:
        return current - max_step
    return target


class BaseController(object):
    def __init__(self):
        n = self.node = CompatNode('galbot_base_controller')
        self.rate = float(n.param('rate', 50.0))
        self.max_v = (float(n.param('max_vx', 0.6)), float(n.param('max_vy', 0.5)),
                      float(n.param('max_wz', 0.9)))
        self.max_acc = (float(n.param('max_acc_lin', 0.8)), float(n.param('max_acc_ang', 1.5)))
        self.timeout = float(n.param('cmd_timeout', 0.5))
        self.publish_tf = bool(n.param('publish_tf', True))
        self.animate = bool(n.param('animate_wheels', True))
        radius = float(n.param('wheel_radius', 0.08273))
        self.swerve = SwerveKinematics(rc.WHEEL_POSITIONS, radius)

        self.lock = threading.Lock()
        self.target = (0.0, 0.0, 0.0)
        self.target_stamp = 0.0
        self.current = [0.0, 0.0, 0.0]
        self._last = None

        self.sim_pub = n.publisher(n.param('output_topic', '/galbot/base/cmd_vel'), Twist, 10)
        self.steer_pub = n.publisher(forward_command_topic('steering_controller'), Float64MultiArray, 10)
        self.wheel_pub = n.publisher(forward_command_topic('wheel_controller'), Float64MultiArray, 10)
        self.odom_pub = n.publisher(n.param('odom_topic', '/odom'), Odometry, 10)
        self.tf = n.tf_broadcaster() if self.publish_tf else None

        n.subscribe(n.param('input_topic', '/cmd_vel'), Twist, self._on_cmd, 10)
        n.subscribe(n.param('ground_truth_topic', '/galbot/ground_truth/odom'), Odometry, self._on_odom, 10)
        n.thread_timer(1.0 / self.rate, self._step, 'base')
        n.info('base controller ready')

    def _on_cmd(self, msg):
        with self.lock:
            self.target = (msg.linear.x, msg.linear.y, msg.angular.z)
            self.target_stamp = time.time()

    def _step(self):
        # acceleration limits are applied in ROS (sim) time so they also hold when
        # the simulator runs slower / faster than real time
        now = self.node.now()
        dt = now - self._last if self._last is not None else 0.0
        self._last = now
        dt = min(max(dt, 0.0), 0.1)
        with self.lock:
            target = self.target if time.time() - self.target_stamp < self.timeout else (0.0, 0.0, 0.0)
        tx = max(-self.max_v[0], min(self.max_v[0], target[0]))
        ty = max(-self.max_v[1], min(self.max_v[1], target[1]))
        tw = max(-self.max_v[2], min(self.max_v[2], target[2]))
        c = self.current
        c[0] = _approach(c[0], tx, self.max_acc[0] * dt)
        c[1] = _approach(c[1], ty, self.max_acc[0] * dt)
        c[2] = _approach(c[2], tw, self.max_acc[1] * dt)
        out = Twist()
        out.linear.x, out.linear.y, out.angular.z = c
        self.sim_pub.publish(out)
        if self.animate:
            angles, speeds = self.swerve.inverse(c[0], c[1], c[2])
            self.steer_pub.publish(Float64MultiArray(data=angles))
            self.wheel_pub.publish(Float64MultiArray(data=speeds))

    def _on_odom(self, msg):
        msg.header.frame_id = msg.header.frame_id or 'odom'
        msg.child_frame_id = msg.child_frame_id or rc.ROBOT_BASE_FRAME
        self.odom_pub.publish(msg)
        if self.tf is not None:
            t = TransformStamped()
            t.header = msg.header
            t.child_frame_id = msg.child_frame_id
            p = msg.pose.pose.position
            t.transform.translation.x = p.x
            t.transform.translation.y = p.y
            t.transform.translation.z = p.z
            t.transform.rotation = msg.pose.pose.orientation
            self.tf.sendTransform(t)


def main():
    BaseController().node.spin()


if __name__ == '__main__':
    main()
