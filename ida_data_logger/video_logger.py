#!/usr/bin/env python3
# =============================================================================
# video_logger.py — Annotated Video Kaydedici
# =============================================================================
# /camera/image_annotated topic'inden annotated kare alır, OpenCV VideoWriter
# ile MP4 dosyasına yazar. Annotated yayın yoksa ham görüntü kullanılır.
# =============================================================================

import os
import datetime

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


_DEFAULT_LOG_DIR = '/logs/video'
_FPS             = 10
_WIDTH           = 640
_HEIGHT          = 480
_FOURCC          = 'mp4v'


class VideoLogger(Node):
    def __init__(self):
        super().__init__('video_logger')

        self.declare_parameter('log_dir',         _DEFAULT_LOG_DIR)
        self.declare_parameter('fps',             float(_FPS))
        self.declare_parameter('width',           _WIDTH)
        self.declare_parameter('height',          _HEIGHT)
        self.declare_parameter('annotated_topic', '/camera/image_annotated')
        self.declare_parameter('raw_topic',       '/camera/image_raw')

        log_dir    = self.get_parameter('log_dir').value
        fps        = self.get_parameter('fps').value
        width      = self.get_parameter('width').value
        height     = self.get_parameter('height').value
        ann_topic  = self.get_parameter('annotated_topic').value
        raw_topic  = self.get_parameter('raw_topic').value

        os.makedirs(log_dir, exist_ok=True)
        ts   = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(log_dir, f'run_{ts}.mp4')

        fourcc = cv2.VideoWriter_fourcc(*_FOURCC)
        self._writer = cv2.VideoWriter(path, fourcc, fps, (width, height))
        if not self._writer.isOpened():
            self.get_logger().error(f'VideoWriter açılamadı: {path}')

        self._bridge = CvBridge()

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
        )

        self._out_size = (int(width), int(height))
        # Annotated (tespit çerçeveli) tercih edilir; annotated geldiyse ham
        # görüntü YOK SAYILIR (aksi halde kareler karışır ve teslim edilen
        # videoda tespitler düzensiz görünür). Ham yayın yalnızca annotated
        # hiç gelmediğinde yedek olarak kullanılır.
        self._annotated_seen = False
        self.create_subscription(Image, ann_topic, self._annotated_cb, sensor_qos)
        self.create_subscription(Image, raw_topic, self._raw_cb, sensor_qos)

        self.get_logger().info(f'Video kaydedici başlatıldı → {path}')

    def _annotated_cb(self, msg: Image):
        self._annotated_seen = True
        self._write(msg)

    def _raw_cb(self, msg: Image):
        if self._annotated_seen:
            return   # annotated akışı var → ham görüntüyü atla
        self._write(msg)

    def _write(self, msg: Image):
        if not self._writer.isOpened():
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # VideoWriter sabit boyut ister; farklıysa yeniden boyutlandır
            # (aksi halde write() sessizce başarısız olur → boş/bozuk mp4).
            if (frame.shape[1], frame.shape[0]) != self._out_size:
                frame = cv2.resize(frame, self._out_size)
            self._writer.write(frame)
        except Exception as e:
            self.get_logger().debug(f'Frame yazma hatası: {e}')

    def destroy_node(self):
        if self._writer.isOpened():
            self._writer.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
