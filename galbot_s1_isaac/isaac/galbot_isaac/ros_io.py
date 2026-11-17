"""rclpy side of the simulator: the same topics as the Gazebo version.

Subscribed
  /<group>_controller/commands   std_msgs/Float64MultiArray  (arms, grippers, torso, head,
                                 steering = positions, wheel = velocities)
  /galbot/base/cmd_vel           geometry_msgs/Twist (base frame, from base_controller.py)
  /galbot/sim/attach             std_msgs/String "<model> <link> <object> [object_link]"
  /galbot/sim/detach             std_msgs/String "<object>" | "*"
  /galbot/sim/set_model_pose     std_msgs/String "<model> x y z roll pitch yaw"
Published
  /clock, /joint_states, /galbot/ground_truth/odom, /galbot/objects/<name>/odom,
  /galbot/sim/attached, /galbot/<camera>/image_raw, /galbot/<camera>/camera_info

Callbacks run in a spin thread and only store data; the simulation loop drains them.
"""
import array
import collections
import queue
import threading

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Float64MultiArray, String


def to_time(t):
    sec = int(t)
    return Time(sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1000000000)


class RosIO(object):
    def __init__(self, controllers, object_names, cameras, node_name='galbot_isaac_sim'):
        """controllers: {controller name: joint count}; cameras: {name: (width, height, K)}."""
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(node_name)
        self.lock = threading.Lock()
        self.commands = {}                      # controller -> list of values (latest)
        self.cmd_seq = collections.Counter()    # controller -> number of messages received
        self.base_cmd = None                    # (vx, vy, wz)
        self.base_seq = 0
        self.requests = collections.deque()     # (kind, text)

        for ctrl in controllers:
            self.node.create_subscription(
                Float64MultiArray, '/%s/commands' % ctrl,
                lambda msg, c=ctrl: self._on_command(c, msg), 10)
        self.node.create_subscription(Twist, '/galbot/base/cmd_vel', self._on_base, 10)
        for kind in ('attach', 'detach', 'set_model_pose'):
            self.node.create_subscription(String, '/galbot/sim/' + kind,
                                          lambda msg, k=kind: self._on_request(k, msg), 20)

        self.clock_pub = self.node.create_publisher(Clock, '/clock', 10)
        self.joint_pub = self.node.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub = self.node.create_publisher(Odometry, '/galbot/ground_truth/odom', 10)
        self.object_pubs = {n: self.node.create_publisher(Odometry, '/galbot/objects/%s/odom' % n, 10)
                            for n in object_names}
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.attached_pub = self.node.create_publisher(String, '/galbot/sim/attached', latched)
        self.cameras = {}
        for name, (width, height, K) in cameras.items():
            self.cameras[name] = {
                'image': self.node.create_publisher(Image, '/galbot/%s/image_raw' % name, 2),
                'info': self.node.create_publisher(CameraInfo, '/galbot/%s/camera_info' % name, 2),
                'width': width, 'height': height, 'K': K}
        self.image_queue = queue.Queue(maxsize=8)
        self.images_published = collections.Counter()
        self.images_dropped = 0

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self._threads = [threading.Thread(target=self._spin, name='ros_spin', daemon=True),
                         threading.Thread(target=self._image_worker, name='ros_images', daemon=True)]
        for t in self._threads:
            t.start()

    # ------------------------------------------------------------------ callbacks
    def _spin(self):
        try:
            self.executor.spin()
        except Exception:  # shutdown
            pass

    def _on_command(self, ctrl, msg):
        with self.lock:
            self.commands[ctrl] = list(msg.data)
            self.cmd_seq[ctrl] += 1

    def _on_base(self, msg):
        with self.lock:
            self.base_cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
            self.base_seq += 1

    def _on_request(self, kind, msg):
        with self.lock:
            self.requests.append((kind, msg.data))

    def snapshot(self):
        """(commands, command seq numbers, base command, base seq, pending requests)."""
        with self.lock:
            reqs = list(self.requests)
            self.requests.clear()
            return dict(self.commands), dict(self.cmd_seq), self.base_cmd, self.base_seq, reqs

    def ok(self):
        """False after Ctrl-C / SIGTERM (rclpy shuts its context down on those signals)."""
        return rclpy.ok()

    def info(self, text):
        self.node.get_logger().info(text)

    def warn(self, text):
        self.node.get_logger().warning(text)

    # ------------------------------------------------------------------ publishers
    def publish_clock(self, t):
        self.clock_pub.publish(Clock(clock=to_time(t)))

    def publish_joints(self, t, names, positions, velocities):
        msg = JointState()
        msg.header.stamp = to_time(t)
        msg.name = list(names)
        msg.position = [float(v) for v in positions]
        msg.velocity = [float(v) for v in velocities]
        self.joint_pub.publish(msg)

    @staticmethod
    def _odom(t, frame, child, position, quat_xyzw, lin=(0.0, 0.0, 0.0), ang=(0.0, 0.0, 0.0)):
        msg = Odometry()
        msg.header.stamp = to_time(t)
        msg.header.frame_id = frame
        msg.child_frame_id = child
        p = msg.pose.pose.position
        p.x, p.y, p.z = [float(v) for v in position]
        o = msg.pose.pose.orientation
        o.x, o.y, o.z, o.w = [float(v) for v in quat_xyzw]
        tw = msg.twist.twist
        tw.linear.x, tw.linear.y, tw.linear.z = [float(v) for v in lin]
        tw.angular.x, tw.angular.y, tw.angular.z = [float(v) for v in ang]
        return msg

    def publish_base(self, t, position, quat_xyzw, lin_body, ang):
        self.odom_pub.publish(self._odom(t, 'odom', 'base_footprint', position, quat_xyzw, lin_body, ang))

    def publish_object(self, t, name, position, quat_xyzw):
        pub = self.object_pubs.get(name)
        if pub is not None:
            pub.publish(self._odom(t, 'odom', name, position, quat_xyzw))

    def publish_attached(self, attached):
        """attached: {object: parent_link} -> "obj:link obj2:link2 " (same text as Gazebo)."""
        self.attached_pub.publish(String(data=''.join('%s:%s ' % kv for kv in sorted(attached.items()))))

    def camera_wanted(self, name):
        cam = self.cameras.get(name)
        return cam is not None and (cam['image'].get_subscription_count() > 0)

    def submit_image(self, t, name, rgb):
        """rgb: HxWx3 uint8 numpy array. Serialized and published by a worker thread."""
        try:
            self.image_queue.put_nowait((t, name, rgb))
        except queue.Full:
            self.images_dropped += 1

    def _image_worker(self):
        while True:
            item = self.image_queue.get()
            if item is None:
                return
            t, name, rgb = item
            cam = self.cameras[name]
            try:
                h, w = rgb.shape[:2]
                msg = Image()
                msg.header.stamp = to_time(t)
                msg.header.frame_id = '%s_optical_frame' % name
                msg.height, msg.width = int(h), int(w)
                msg.encoding = 'rgb8'
                msg.is_bigendian = 0
                msg.step = int(w) * 3
                data = array.array('B')
                data.frombytes(rgb.tobytes())
                msg.data = data
                cam['image'].publish(msg)
                info = CameraInfo()
                info.header = msg.header
                info.height, info.width = msg.height, msg.width
                info.distortion_model = 'plumb_bob'
                info.d = [0.0] * 5
                info.k = list(cam['K'])
                info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                fx, cx, fy, cy = cam['K'][0], cam['K'][2], cam['K'][4], cam['K'][5]
                info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
                cam['info'].publish(info)
                self.images_published[name] += 1
            except Exception as exc:  # never kill the worker
                self.warn('image %s: %s' % (name, exc))

    def shutdown(self):
        try:
            self.image_queue.put_nowait(None)
        except queue.Full:
            pass
        try:
            self.executor.shutdown(timeout_sec=1.0)
        except Exception:
            pass
        try:
            self.node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
