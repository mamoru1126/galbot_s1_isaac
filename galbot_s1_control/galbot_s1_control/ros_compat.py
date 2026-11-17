"""Tiny ROS 1 / ROS 2 compatibility layer.

The nodes of this package are written once against `CompatNode` and run on
rospy (ROS 1 Noetic) or rclpy (ROS 2) depending on the ROS_VERSION environment
variable that every ROS setup script exports.

Threading model (same for both):
  * subscriber / timer callbacks run in background threads
  * the node's main thread is free for long running logic (demo tasks)
"""
import array
import os
import threading
import time

ROS_VERSION = int(os.environ.get('ROS_VERSION', '2'))

if ROS_VERSION == 1:
    import rospy
    import tf2_ros
else:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
    import tf2_ros


def forward_command_topic(controller):
    """forward_command_controller topic name (ROS1: ~command, ROS2: ~commands)."""
    return '/%s/%s' % (controller, 'command' if ROS_VERSION == 1 else 'commands')


def to_uint8_array(data):
    """Value suitable for a uint8[] message field."""
    if ROS_VERSION == 1:
        return bytes(data)
    return array.array('B', bytes(data))


def stamp_to_sec(stamp):
    if ROS_VERSION == 1:
        return stamp.to_sec()
    return stamp.sec + stamp.nanosec * 1e-9


class CompatNode(object):
    def __init__(self, name):
        self.name = name
        self._shutdown = False
        if ROS_VERSION == 1:
            rospy.init_node(name)
            self._node = None
        else:
            rclpy.init()
            self._node = rclpy.create_node(
                name, automatically_declare_parameters_from_overrides=True)
            self._executor = MultiThreadedExecutor(num_threads=4)
            self._executor.add_node(self._node)
            self._spin_thread = threading.Thread(target=self._spin, daemon=True)
            self._spin_thread.start()

    # ------------------------------------------------------------------ basics
    def _spin(self):
        try:
            self._executor.spin()
        except Exception:  # shutdown race
            pass

    @property
    def raw(self):
        """rclpy Node (ROS 2) or None (ROS 1)."""
        return self._node

    def ok(self):
        if self._shutdown:
            return False
        if ROS_VERSION == 1:
            return not rospy.is_shutdown()
        return rclpy.ok()

    def spin(self):
        """Block until shutdown."""
        try:
            while self.ok():
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        self.shutdown()

    def shutdown(self):
        if self._shutdown:
            return
        self._shutdown = True
        if ROS_VERSION == 1:
            rospy.signal_shutdown('done')
        else:
            try:
                self._executor.shutdown(timeout_sec=1.0)
                self._node.destroy_node()
                rclpy.try_shutdown()
            except Exception:
                pass

    # ------------------------------------------------------------------ params
    def param(self, name, default):
        if ROS_VERSION == 1:
            if rospy.has_param('~' + name):
                return rospy.get_param('~' + name)
            if rospy.has_param('/' + name):
                return rospy.get_param('/' + name)
            return default
        if not self._node.has_parameter(name):
            try:
                self._node.declare_parameter(name, default)
            except Exception:
                return default
        value = self._node.get_parameter(name).value
        return default if value is None else value

    # ------------------------------------------------------------------ logging
    def info(self, msg, *args):
        msg = msg % args if args else msg
        if ROS_VERSION == 1:
            rospy.loginfo(msg)
        else:
            self._node.get_logger().info(msg)

    def warn(self, msg, *args):
        msg = msg % args if args else msg
        if ROS_VERSION == 1:
            rospy.logwarn(msg)
        else:
            self._node.get_logger().warning(msg)

    def error(self, msg, *args):
        msg = msg % args if args else msg
        if ROS_VERSION == 1:
            rospy.logerr(msg)
        else:
            self._node.get_logger().error(msg)

    # ------------------------------------------------------------------ time
    def now(self):
        """Current ROS time (sim time if use_sim_time) in seconds."""
        if ROS_VERSION == 1:
            return rospy.get_time()
        return self._node.get_clock().now().nanoseconds * 1e-9

    def stamp(self):
        if ROS_VERSION == 1:
            return rospy.Time.now()
        return self._node.get_clock().now().to_msg()

    def sleep(self, seconds):
        """Sleep in ROS time; falls back to wall time while the clock is not running."""
        if seconds <= 0:
            return
        start = self.now()
        wall_start = time.time()
        while self.ok():
            now = self.now()
            if now > 0.0 and now - start >= seconds:
                return
            # clock not running (e.g. sim paused / not started) -> wall clock limit
            if now <= 0.0 and time.time() - wall_start >= seconds:
                return
            if time.time() - wall_start > 50.0 * seconds + 30.0:
                return
            time.sleep(0.005)

    def wait_for(self, predicate, timeout, period=0.02):
        """Wait (ROS time) until predicate() is true. Returns predicate result."""
        start = self.now()
        wall_start = time.time()
        while self.ok():
            if predicate():
                return True
            now = self.now()
            if timeout is not None:
                if now > 0.0 and now - start > timeout:
                    return bool(predicate())
                if time.time() - wall_start > 10.0 * timeout + 30.0:
                    return bool(predicate())
            time.sleep(period)
        return False

    # ------------------------------------------------------------------ pub/sub
    def publisher(self, topic, msg_type, queue_size=10, latch=False):
        if ROS_VERSION == 1:
            return _Pub1(rospy.Publisher(topic, msg_type, queue_size=queue_size, latch=latch))
        qos = QoSProfile(depth=queue_size)
        if latch:
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        return _Pub2(self._node.create_publisher(msg_type, topic, qos))

    def subscribe(self, topic, msg_type, callback, queue_size=10):
        if ROS_VERSION == 1:
            return rospy.Subscriber(topic, msg_type, callback, queue_size=queue_size)
        return self._node.create_subscription(msg_type, topic, callback, queue_size)

    def timer(self, period, callback):
        if ROS_VERSION == 1:
            return rospy.Timer(rospy.Duration(period), lambda _evt: callback())
        return self._node.create_timer(period, callback)

    def thread_timer(self, period, callback, name=None):
        """Wall-clock periodic thread (keeps running when sim time is paused)."""
        def loop():
            next_t = time.time()
            while self.ok():
                try:
                    callback()
                except Exception as exc:  # keep the loop alive
                    self.error('%s: %s', name or 'timer', exc)
                next_t += period
                delay = next_t - time.time()
                if delay < -period:
                    next_t = time.time()
                    delay = 0
                time.sleep(max(0.0, delay))
        t = threading.Thread(target=loop, daemon=True, name=name)
        t.start()
        return t

    def tf_broadcaster(self):
        if ROS_VERSION == 1:
            return tf2_ros.TransformBroadcaster()
        return tf2_ros.TransformBroadcaster(self._node)


class _Pub1(object):
    def __init__(self, pub):
        self._pub = pub

    def publish(self, msg):
        self._pub.publish(msg)

    def subscriber_count(self):
        return self._pub.get_num_connections()


class _Pub2(object):
    def __init__(self, pub):
        self._pub = pub

    def publish(self, msg):
        self._pub.publish(msg)

    def subscriber_count(self):
        return self._pub.get_subscription_count()
