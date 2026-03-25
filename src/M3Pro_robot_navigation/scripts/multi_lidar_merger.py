#!/usr/bin/env python3
import math

import message_filters
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class MultiLidarMerger(Node):
    def __init__(self):
        super().__init__('multi_lidar_merger')

        self._declare_parameters()

        self.front_topic = self.get_parameter('front_topic').value
        self.rear_topic = self.get_parameter('rear_topic').value
        self.merged_topic = self.get_parameter('merged_topic').value

        self.output_frame = self.get_parameter('output_frame').value

        self.angle_min = float(self.get_parameter('angle_min').value)
        self.angle_max = float(self.get_parameter('angle_max').value)
        self.angle_increment = float(self.get_parameter('angle_increment').value)
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)

        self.sync_queue_size = int(self.get_parameter('sync_queue_size').value)
        self.sync_slop = float(self.get_parameter('sync_slop').value)

        self.front_offset = {
            'x': float(self.get_parameter('front_offset_x').value),
            'y': float(self.get_parameter('front_offset_y').value),
            'yaw': float(self.get_parameter('front_yaw').value),
        }
        self.rear_offset = {
            'x': float(self.get_parameter('rear_offset_x').value),
            'y': float(self.get_parameter('rear_offset_y').value),
            'yaw': float(self.get_parameter('rear_yaw').value),
        }

        raw_bin_count = (self.angle_max - self.angle_min) / self.angle_increment
        self.bin_count = max(1, int(round(raw_bin_count)))

        self.sub_front = message_filters.Subscriber(
            self,
            LaserScan,
            self.front_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.sub_rear = message_filters.Subscriber(
            self,
            LaserScan,
            self.rear_topic,
            qos_profile=qos_profile_sensor_data,
        )

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_front, self.sub_rear],
            self.sync_queue_size,
            self.sync_slop,
        )
        self.ts.registerCallback(self.merge_callback)

        self.pub_merged = self.create_publisher(LaserScan, self.merged_topic, 10)

        self.get_logger().info(
            'multi_lidar_merger started: '
            f'front={self.front_topic}, rear={self.rear_topic}, merged={self.merged_topic}, '
            f'frame={self.output_frame}, bins={self.bin_count}'
        )
        self.get_logger().info(
            f'front_offset={self.front_offset}, rear_offset={self.rear_offset}, '
            f'sync(queue={self.sync_queue_size}, slop={self.sync_slop})'
        )

    def _declare_parameters(self):
        self.declare_parameter('front_topic', '/scan')
        self.declare_parameter('rear_topic', '/scan_back')
        self.declare_parameter('merged_topic', '/scan_merged')

        self.declare_parameter('output_frame', 'base_footprint')

        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('angle_increment', math.pi / 360.0)
        self.declare_parameter('range_min', 0.2)
        self.declare_parameter('range_max', 12.0)

        self.declare_parameter('sync_queue_size', 10)
        self.declare_parameter('sync_slop', 0.05)

        self.declare_parameter('front_offset_x', -0.11617)
        self.declare_parameter('front_offset_y', 0.09156)
        self.declare_parameter('front_yaw', 0.0)

        self.declare_parameter('rear_offset_x', 0.10766)
        self.declare_parameter('rear_offset_y', -0.09078)
        self.declare_parameter('rear_yaw', 0.0)

    def merge_callback(self, front_msg: LaserScan, rear_msg: LaserScan):
        merged = LaserScan()
        merged.header.stamp = front_msg.header.stamp
        merged.header.frame_id = self.output_frame

        merged.angle_min = self.angle_min
        merged.angle_max = self.angle_max
        merged.angle_increment = self.angle_increment
        merged.range_min = self.range_min
        merged.range_max = self.range_max
        merged.scan_time = max(front_msg.scan_time, rear_msg.scan_time)
        merged.time_increment = max(front_msg.time_increment, rear_msg.time_increment)

        ranges = [float('inf')] * self.bin_count

        self._project_scan(front_msg, self.front_offset, merged, ranges)
        self._project_scan(rear_msg, self.rear_offset, merged, ranges)

        merged.ranges = ranges
        merged.intensities = []
        self.pub_merged.publish(merged)

    def _project_scan(self, msg: LaserScan, offset, merged: LaserScan, ranges):
        cos_y = math.cos(offset['yaw'])
        sin_y = math.sin(offset['yaw'])

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            if r < msg.range_min or r > msg.range_max:
                continue

            local_theta = msg.angle_min + i * msg.angle_increment

            lx = r * math.cos(local_theta)
            ly = r * math.sin(local_theta)

            bx = lx * cos_y - ly * sin_y + offset['x']
            by = lx * sin_y + ly * cos_y + offset['y']

            new_r = math.hypot(bx, by)
            if new_r < self.range_min or new_r > self.range_max:
                continue

            new_theta = math.atan2(by, bx)
            idx = int((new_theta - merged.angle_min) / merged.angle_increment)

            if 0 <= idx < self.bin_count and new_r < ranges[idx]:
                ranges[idx] = new_r


def main(args=None):
    rclpy.init(args=args)
    node = MultiLidarMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
