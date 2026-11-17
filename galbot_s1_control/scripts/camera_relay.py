#!/usr/bin/env python3
"""Re-publish camera images as small JPEGs for the web UI (ROS 1 / ROS 2).

sensor_msgs/Image  <camera topic>  ->  sensor_msgs/CompressedImage  /galbot/web/<name>/compressed

rosbridge sends uint8[] as base64, so the browser can show the JPEG directly
without web_video_server. Frames are only encoded while somebody subscribes.
"""
import threading

import numpy as np
from sensor_msgs.msg import CompressedImage, Image

from galbot_s1_control.ros_compat import CompatNode, to_uint8_array

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None
try:
    from PIL import Image as PILImage
    import io
except ImportError:  # pragma: no cover
    PILImage = None


def image_to_rgb(msg):
    enc = msg.encoding.lower()
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    rows = buf.reshape(msg.height, msg.step)
    if enc in ('rgb8', 'bgr8'):
        img = rows[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        return img[:, :, ::-1] if enc == 'bgr8' else img
    if enc in ('rgba8', 'bgra8'):
        img = rows[:, :msg.width * 4].reshape(msg.height, msg.width, 4)[:, :, :3]
        return img[:, :, ::-1] if enc == 'bgra8' else img
    if enc in ('mono8', '8uc1'):
        img = rows[:, :msg.width]
        return np.repeat(img[:, :, None], 3, axis=2)
    raise ValueError('unsupported encoding %s' % msg.encoding)


def encode_jpeg(rgb, max_width, quality):
    h, w = rgb.shape[:2]
    if cv2 is not None:
        img = rgb[:, :, ::-1]
        if w > max_width:
            img = cv2.resize(img, (max_width, int(h * max_width / float(w))), interpolation=cv2.INTER_AREA)
        ok, data = cv2.imencode('.jpg', np.ascontiguousarray(img), [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
        return data.tobytes() if ok else None
    if PILImage is not None:
        im = PILImage.fromarray(np.ascontiguousarray(rgb))
        if w > max_width:
            im = im.resize((max_width, int(h * max_width / float(w))))
        out = io.BytesIO()
        im.save(out, format='JPEG', quality=int(quality))
        return out.getvalue()
    return None


class CameraRelay(object):
    def __init__(self):
        n = self.node = CompatNode('galbot_camera_relay')
        spec = n.param('cameras', 'overview:/galbot/overview_camera/image_raw,'
                                  'head:/galbot/head_camera/image_raw,'
                                  'right_wrist:/galbot/right_wrist_camera/image_raw,'
                                  'left_wrist:/galbot/left_wrist_camera/image_raw')
        self.max_width = int(n.param('max_width', 480))
        self.quality = int(n.param('jpeg_quality', 70))
        rate = float(n.param('rate', 8.0))
        prefix = n.param('output_prefix', '/galbot/web')
        if cv2 is None and PILImage is None:
            n.error('neither OpenCV (python3-opencv) nor Pillow is installed - camera relay disabled')
        self.lock = threading.Lock()
        self.cams = {}
        for item in spec.split(','):
            item = item.strip()
            if not item or ':' not in item:
                continue
            name, topic = item.split(':', 1)
            cam = {'latest': None, 'seq': 0, 'sent': 0,
                   'pub': n.publisher('%s/%s/compressed' % (prefix, name), CompressedImage, 2)}
            self.cams[name] = cam
            n.subscribe(topic, Image, lambda msg, c=cam: self._on_image(c, msg), 2)
        n.thread_timer(1.0 / rate, self._step, 'camera_relay')
        n.info('camera relay: %s' % ', '.join(self.cams))

    def _on_image(self, cam, msg):
        with self.lock:
            cam['latest'] = msg
            cam['seq'] += 1

    def _step(self):
        for name, cam in self.cams.items():
            with self.lock:
                msg, seq = cam['latest'], cam['seq']
            if msg is None or seq == cam['sent'] or cam['pub'].subscriber_count() == 0:
                continue
            cam['sent'] = seq
            try:
                data = encode_jpeg(image_to_rgb(msg), self.max_width, self.quality)
            except Exception as exc:
                self.node.warn('%s: %s' % (name, exc))
                continue
            if data is None:
                continue
            out = CompressedImage()
            out.header = msg.header
            out.format = 'jpeg'
            out.data = to_uint8_array(data)
            cam['pub'].publish(out)


def main():
    CameraRelay().node.spin()


if __name__ == '__main__':
    main()
