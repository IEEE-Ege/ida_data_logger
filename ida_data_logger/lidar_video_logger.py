#!/usr/bin/env python3
# =============================================================================
# lidar_video_logger.py — LiDAR Kuşbakışı MP4 Kaydedici (Şartname Dosya 1)
# =============================================================================
# Şartname 4.2 "Dosya 1 → Diğer Otonomi Sensörleri Veri Seti":
#   • Kamera dışı her otonomi sensörü (LiDAR) için ayrı,
#   • En az 1 Hz,
#   • Her veri seti zaman etiketli,
#   • mp4 formatında,
#   • "Tespit ve takip işlemleri sonucunda kümeleme, ayırma vs. gibi bir işlem
#      yapıldıysa görünecek şekilde".
#
# Bu düğüm işlenmiş LiDAR bulutunu (/lidar/filtered) kuşbakışı (top-down)
# bir görüntüye çizer, üstüne füzyon sonucu tespit edilen şamandıraları
# (/buoy_detections) renk/etiketle işaretler, her kareye zaman damgası basar
# ve MP4 olarak yazar.
# =============================================================================

import datetime
import math
import os

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from ida_msgs.msg import BuoyDetectionArray


_DEFAULT_LOG_DIR = '/mnt/sdcard/ida_logs/lidar'

_COLOR_BGR = {
    'red':    (0, 0, 255),
    'green':  (0, 200, 0),
    'orange': (0, 140, 255),
    'yellow': (0, 220, 220),
    'white':  (230, 230, 230),
    'black':  (40, 40, 40),
}


class LidarVideoLogger(Node):
    def __init__(self):
        super().__init__('lidar_video_logger')

        self.declare_parameter('log_dir', _DEFAULT_LOG_DIR)
        self.declare_parameter('fps', 5.0)
        self.declare_parameter('size_px', 720)
        self.declare_parameter('range_m', 35.0)     # yarı-genişlik (m)
        self.declare_parameter('point_topic', '/lidar/filtered')
        self.declare_parameter('buoy_topic', '/buoy_detections')

        log_dir      = self.get_parameter('log_dir').value
        self._fps    = float(self.get_parameter('fps').value)
        self._sz     = int(self.get_parameter('size_px').value)
        self._range  = float(self.get_parameter('range_m').value)
        point_topic  = self.get_parameter('point_topic').value
        buoy_topic   = self.get_parameter('buoy_topic').value

        os.makedirs(log_dir, exist_ok=True)
        ts   = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(log_dir, f'lidar_{ts}.mp4')
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self._writer = cv2.VideoWriter(path, fourcc, self._fps, (self._sz, self._sz))
        if not self._writer.isOpened():
            self.get_logger().error(f'VideoWriter açılamadı: {path}')

        self._scale = (self._sz / 2.0) / max(self._range, 1e-3)  # px/m
        self._latest_pts = np.empty((0, 2), dtype=np.float32)
        self._latest_buoys = []

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
        )
        self.create_subscription(PointCloud2, point_topic,
                                 self._cloud_cb, sensor_qos)
        self.create_subscription(BuoyDetectionArray, buoy_topic,
                                 self._buoy_cb, sensor_qos)

        self.create_timer(1.0 / max(self._fps, 1.0), self._render)
        self.get_logger().info(
            f'LiDAR video kaydedici başlatıldı ({self._fps} Hz) → {path}')

    # ─────────────────────────────────────────────────────────────────────────

    def _cloud_cb(self, msg: PointCloud2):
        try:
            # read_points: Humble'da her zaman mevcut (structured ndarray ya da
            # generator). Her iki durumda da satır satır iterasyon x/y verir.
            data = point_cloud2.read_points(
                msg, field_names=('x', 'y'), skip_nans=True)
            pts = np.array([[p[0], p[1]] for p in data], dtype=np.float32)
            self._latest_pts = pts.reshape(-1, 2) if pts.size else \
                np.empty((0, 2), dtype=np.float32)
        except Exception as e:
            self.get_logger().debug(f'Bulut okunamadı: {e}')

    def _buoy_cb(self, msg: BuoyDetectionArray):
        self._latest_buoys = list(msg.buoys)

    def _world_to_px(self, x, y):
        # Kuşbakışı: +X ileri = yukarı, +Y sol = sola
        u = int(self._sz / 2.0 - y * self._scale)
        v = int(self._sz / 2.0 - x * self._scale)
        return u, v

    def _render(self):
        if not self._writer.isOpened():
            return
        img = np.zeros((self._sz, self._sz, 3), dtype=np.uint8)

        # Menzil halkaları + eksenler
        c = self._sz // 2
        for r_m in range(10, int(self._range) + 1, 10):
            cv2.circle(img, (c, c), int(r_m * self._scale), (40, 40, 40), 1)
        cv2.line(img, (c, 0), (c, self._sz), (30, 30, 30), 1)
        cv2.line(img, (0, c), (self._sz, c), (30, 30, 30), 1)

        # Nokta bulutu
        pts = self._latest_pts
        if pts.shape[0] > 0:
            u = (c - pts[:, 1] * self._scale).astype(np.int32)
            v = (c - pts[:, 0] * self._scale).astype(np.int32)
            m = (u >= 0) & (u < self._sz) & (v >= 0) & (v < self._sz)
            img[v[m], u[m]] = (0, 200, 255)

        # Araç (merkez)
        cv2.drawMarker(img, (c, c), (255, 255, 255), cv2.MARKER_TRIANGLE_UP, 14, 2)

        # Tespit edilen şamandıralar (kümeleme/füzyon sonucu)
        for b in self._latest_buoys:
            u, v = self._world_to_px(b.position.x, b.position.y)
            if not (0 <= u < self._sz and 0 <= v < self._sz):
                continue
            col = _COLOR_BGR.get(getattr(b, 'color', 'white'), (230, 230, 230))
            cv2.circle(img, (u, v), 8, col, 2)
            label = f'{getattr(b, "color", "?")} {getattr(b, "confidence", 0.0):.2f}'
            cv2.putText(img, label, (u + 10, v),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1, cv2.LINE_AA)

        # Zaman damgası (her kare — şartname: "zaman etiketine sahip")
        stamp = datetime.datetime.utcnow().isoformat()
        cv2.putText(img, stamp, (8, self._sz - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(img, f'range +/-{int(self._range)} m', (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 150), 1, cv2.LINE_AA)

        self._writer.write(img)

    def destroy_node(self):
        if self._writer.isOpened():
            self._writer.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarVideoLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
