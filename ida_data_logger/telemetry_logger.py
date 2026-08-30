#!/usr/bin/env python3
# =============================================================================
# telemetry_logger.py — Araç Telemetri CSV Kaydedici (Şartname Dosya 2)
# =============================================================================
# Şartname 4.2 "Faydalı Yük ve Otonomi → Dosya 2: Araç telemetri verisi":
#   • En az 1 Hz
#   • Konum (lat, lon)
#   • Hız (yer hızı)
#   • Yönelim açıları (roll, pitch, heading)
#   • Hız set pointi
#   • Yön set pointi
#   • csv formatı, ilk satır header
#
# Kaynaklar:
#   /mavros/global_position/global (NavSatFix)   → lat, lon
#   /mavros/local_position/odom    (Odometry)    → yer hızı + roll/pitch/heading
#   /cmd_vel                       (Twist)       → hız set pointi + yön set pointi
#   /mission/state                 (MissionState)→ görev durumu (referans sütun)
#
# "Yön set pointi": araç gövde-çerçevesi hız kontrolüyle (GUIDED velocity)
# sürüldüğü için mutlak yön komutu yerine, komut edilen yaw hızından kısa
# ufuklu (heading_setpoint_horizon_sec) ileri-yansıtılmış hedef başlık olarak
# hesaplanır: heading_sp = wrap(heading + yaw_rate · horizon).
# =============================================================================

import csv
import datetime
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from ida_msgs.msg import MissionState


# Varsayılan: harici SD kart (şartname kayıtları harici depolamada tutulur).
# Gerçek yol logger_params.yaml içinden verilir; SD yoksa entrypoint iç diske
# düşer (bkz. kurulum.md).
_DEFAULT_LOG_DIR = '/mnt/sdcard/ida_logs/telemetry'


def _quat_to_euler(x, y, z, w):
    """Quaternion → (roll, pitch, yaw) radyan. ZYX konvansiyonu."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _wrap_deg(a):
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


class TelemetryLogger(Node):
    def __init__(self):
        super().__init__('telemetry_logger')

        self.declare_parameter('log_dir', _DEFAULT_LOG_DIR)
        self.declare_parameter('rate_hz', 2.0)   # ≥1 Hz şartı; 2 Hz pay bırakır
        self.declare_parameter('heading_setpoint_horizon_sec', 1.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        log_dir = self.get_parameter('log_dir').value
        rate    = float(self.get_parameter('rate_hz').value)
        self._hz_horizon = float(
            self.get_parameter('heading_setpoint_horizon_sec').value)
        cmd_topic = self.get_parameter('cmd_vel_topic').value

        os.makedirs(log_dir, exist_ok=True)
        ts   = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(log_dir, f'telemetry_{ts}.csv')
        self._file   = open(path, 'w', newline='')
        self._writer = csv.writer(self._file)
        # Şartname Dosya 2 header'ı (ilk satır)
        self._writer.writerow([
            'timestamp',
            'latitude', 'longitude',
            'ground_speed_mps',
            'roll_deg', 'pitch_deg', 'heading_deg',
            'speed_setpoint_mps', 'heading_setpoint_deg',
            'mission_state',
        ])
        self._file.flush()

        # ── Durum önbelleği ────────────────────────────────────────────────
        self._lat = 0.0
        self._lon = 0.0
        self._speed = 0.0
        self._roll_deg = 0.0
        self._pitch_deg = 0.0
        self._heading_deg = 0.0
        self._speed_sp = 0.0     # /cmd_vel linear.x
        self._yaw_rate_sp = 0.0  # /cmd_vel angular.z (rad/s)
        self._mstate = 'IDLE'

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.create_subscription(NavSatFix, '/mavros/global_position/global',
                                 self._gps_cb, sensor_qos)
        self.create_subscription(Odometry, '/mavros/local_position/odom',
                                 self._odom_cb, sensor_qos)
        self.create_subscription(Twist, cmd_topic, self._cmd_cb, 10)
        self.create_subscription(MissionState, '/mission/state',
                                 self._mission_cb, 10)

        self.create_timer(1.0 / max(rate, 1.0), self._log)
        self.get_logger().info(
            f'Telemetri kaydedici başlatıldı ({rate} Hz) → {path}')

    # ─────────────────────────────────────────────────────────────────────────

    def _gps_cb(self, msg: NavSatFix):
        self._lat = msg.latitude
        self._lon = msg.longitude

    def _odom_cb(self, msg: Odometry):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self._speed = math.hypot(vx, vy)   # yer hızı
        q = msg.pose.pose.orientation
        roll, pitch, yaw = _quat_to_euler(q.x, q.y, q.z, q.w)
        self._roll_deg    = math.degrees(roll)
        self._pitch_deg   = math.degrees(pitch)
        self._heading_deg = math.degrees(yaw)

    def _cmd_cb(self, msg: Twist):
        self._speed_sp    = msg.linear.x
        self._yaw_rate_sp = msg.angular.z

    def _mission_cb(self, msg: MissionState):
        self._mstate = msg.state

    def _log(self):
        now = datetime.datetime.utcnow().isoformat()
        # Yön set pointi: ölçülen başlık + komut yaw hızı × kısa ufuk
        heading_sp = _wrap_deg(
            self._heading_deg + math.degrees(self._yaw_rate_sp) * self._hz_horizon)
        self._writer.writerow([
            now,
            f'{self._lat:.8f}',
            f'{self._lon:.8f}',
            f'{self._speed:.3f}',
            f'{self._roll_deg:.2f}',
            f'{self._pitch_deg:.2f}',
            f'{self._heading_deg:.2f}',
            f'{self._speed_sp:.3f}',
            f'{heading_sp:.2f}',
            self._mstate,
        ])
        self._file.flush()

    def destroy_node(self):
        try:
            self._file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
