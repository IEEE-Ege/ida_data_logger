#!/usr/bin/env python3
# =============================================================================
# costmap_logger.py — Costmap ROS Bag Kaydedici
# =============================================================================
# Nav2 Humble OccupancyGrid topic'lerini rosbag2 ile kaydeder.
# Hangi topic'in yayınlandığı kurulum/sürüme göre değişebilir:
#   /local_costmap/costmap
#   /local_costmap/costmap_raw
#   /global_costmap/costmap
# Bu logger, parametre olarak verilen topic listesinin her birini
# bağımsız subscriber + ayrı bag topic olarak kaydeder.
# =============================================================================

import datetime
import os
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import rosbag2_py
from rclpy.serialization import serialize_message
from nav_msgs.msg import OccupancyGrid


_DEFAULT_LOG_DIR = '/logs/bags'
_DEFAULT_TOPICS  = [
    '/local_costmap/costmap',
    '/local_costmap/costmap_raw',
    '/global_costmap/costmap',
]


class CostmapLogger(Node):
    def __init__(self):
        super().__init__('costmap_logger')

        self.declare_parameter('log_dir', _DEFAULT_LOG_DIR)
        self.declare_parameter('topics',  _DEFAULT_TOPICS)
        log_dir = self.get_parameter('log_dir').value
        topics: List[str] = list(self.get_parameter('topics').value)

        os.makedirs(log_dir, exist_ok=True)
        ts       = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        bag_path = os.path.join(log_dir, f'costmap_{ts}')

        self._writer = rosbag2_py.SequentialWriter()
        storage_opts = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
        conv_opts    = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        )
        self._writer.open(storage_opts, conv_opts)

        # Nav2 costmap topic'leri TRANSIENT_LOCAL + RELIABLE QoS ile yayınlanır
        nav2_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._active_topics = set()

        for topic in topics:
            self._writer.create_topic(rosbag2_py.TopicMetadata(
                name=topic,
                type='nav_msgs/msg/OccupancyGrid',
                serialization_format='cdr',
            ))
            self.create_subscription(
                OccupancyGrid, topic,
                self._make_callback(topic), nav2_qos)

        self.get_logger().info(
            f'Costmap logger başlatıldı → {bag_path}\n'
            f'Dinlenen topic\'ler: {topics}'
        )

    def _make_callback(self, topic: str):
        def cb(msg: OccupancyGrid):
            if topic not in self._active_topics:
                self._active_topics.add(topic)
                self.get_logger().info(f'İlk mesaj alındı: {topic}')
            stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            self._writer.write(topic, serialize_message(msg), stamp_ns)
        return cb

    def destroy_node(self):
        del self._writer
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CostmapLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
