"""Small pose helpers (numpy only).

Conventions: quaternions are (x, y, z, w) like ROS unless the name says ``wxyz``
(Isaac Sim / USD order). Poses are 4x4 homogeneous matrices.
"""
import math

import numpy as np


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def matrix_to_quat(R):
    """Rotation matrix -> (x, y, z, w), w >= 0."""
    m = np.asarray(R, dtype=float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w, x, y, z = 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w, x, y, z = (m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w, x, y, z = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w, x, y, z = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    q /= np.linalg.norm(q)
    return q if q[3] >= 0 else -q


def quat_to_matrix(q):
    x, y, z, w = [float(v) for v in q]
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)]])


def xyzw_to_wxyz(q):
    return np.array([q[3], q[0], q[1], q[2]], dtype=float)


def wxyz_to_xyzw(q):
    return np.array([q[1], q[2], q[3], q[0]], dtype=float)


def make_pose(position, R=None):
    T = np.eye(4)
    T[:3, 3] = position
    if R is not None:
        T[:3, :3] = R
    return T


def pose_from_pq(position, quat_xyzw):
    return make_pose(position, quat_to_matrix(quat_xyzw))


def pose_to_pq(T):
    return np.array(T[:3, 3], dtype=float), matrix_to_quat(T[:3, :3])


def planar_pose(x, y, yaw, z=0.0):
    return make_pose((x, y, z), rpy_to_matrix(0.0, 0.0, yaw))


def yaw_quat(yaw):
    return np.array([0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)])


def yaw_of_matrix(R):
    return math.atan2(R[1, 0], R[0, 0])


def wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# A ROS camera body frame (x forward, y left, z up) -> USD camera frame (looks along -Z,
# +Y up, +X right). Columns = USD camera axes expressed in the ROS body frame.
ROS_BODY_TO_USD_CAMERA = np.array([
    [0.0, 0.0, -1.0],
    [-1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0]])


def focal_length_for_hfov(hfov, horizontal_aperture):
    """USD camera focal length (same unit as the aperture) for a horizontal field of view."""
    return horizontal_aperture / (2.0 * math.tan(hfov / 2.0))


def camera_intrinsics(width, height, hfov):
    """Pinhole K (square pixels) for sensor_msgs/CameraInfo."""
    fx = width / (2.0 * math.tan(hfov / 2.0))
    return [fx, 0.0, width / 2.0, 0.0, fx, height / 2.0, 0.0, 0.0, 1.0]
