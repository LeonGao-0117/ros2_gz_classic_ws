#!/usr/bin/env python3
import math
import os
import sys
from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
UTIL_PATHS = [
    os.path.join(SCRIPT_DIR, 'utils'),
    os.path.join(SCRIPT_DIR, 'scripts', 'utils'),
]
for util_path in UTIL_PATHS:
    if os.path.isdir(util_path) and util_path not in sys.path:
        sys.path.append(util_path)

from waypoint_io import load_waypoints  # noqa: E402


def yaw_to_quat(yaw_rad: float):
    half = yaw_rad * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class WaypointMarkerPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_marker_publisher')

        self.declare_parameter('waypoints_file', 'src/M3Pro_robot_navigation/config/waypoints.json')
        self.declare_parameter('marker_topic', '/named_waypoints')
        self.declare_parameter('poll_interval', 1.0)
        self.declare_parameter('text_offset_z', 0.3)
        self.declare_parameter('frame_id_fallback', 'map')

        self.waypoints_file = self.get_parameter('waypoints_file').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.poll_interval = float(self.get_parameter('poll_interval').value)
        self.text_offset_z = float(self.get_parameter('text_offset_z').value)
        self.frame_id_fallback = self.get_parameter('frame_id_fallback').value

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.pub = self.create_publisher(MarkerArray, self.marker_topic, qos)

        self._last_mtime = None

        self.get_logger().info(
            f'Publishing named waypoints to {self.marker_topic} from {os.path.abspath(self.waypoints_file)}'
        )

        self._publish_if_changed(force=True)
        self.create_timer(max(self.poll_interval, 0.1), self._on_timer)

    def _on_timer(self):
        self._publish_if_changed(force=False)

    def _current_mtime(self):
        path = os.path.abspath(self.waypoints_file)
        if not os.path.exists(path):
            return None
        try:
            return os.path.getmtime(path)
        except OSError:
            return None

    def _publish_if_changed(self, force: bool):
        current = self._current_mtime()
        if not force and current == self._last_mtime:
            return

        table = load_waypoints(self.waypoints_file)
        waypoints = table.get('waypoints', {})
        if not isinstance(waypoints, dict):
            waypoints = {}

        marker_array = self._build_marker_array(waypoints)
        self.pub.publish(marker_array)
        self._last_mtime = current
        self.get_logger().info(f'Published {len(waypoints)} waypoint(s) to {self.marker_topic}')

    def _build_marker_array(self, waypoints: Dict[str, dict]) -> MarkerArray:
        msg = MarkerArray()

        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        msg.markers.append(delete_all)

        now = self.get_clock().now().to_msg()
        for i, name in enumerate(sorted(waypoints.keys())):
            wp = waypoints.get(name, {})
            frame_id = str(wp.get('frame_id', self.frame_id_fallback))

            x = float(wp.get('x', 0.0))
            y = float(wp.get('y', 0.0))
            z = float(wp.get('z', 0.0))

            if all(k in wp for k in ('qx', 'qy', 'qz', 'qw')):
                qx = float(wp.get('qx', 0.0))
                qy = float(wp.get('qy', 0.0))
                qz = float(wp.get('qz', 0.0))
                qw = float(wp.get('qw', 1.0))
            else:
                yaw = float(wp.get('yaw_rad', math.radians(float(wp.get('yaw_deg', 0.0)))))
                qx, qy, qz, qw = yaw_to_quat(yaw)

            base_id = 3 * i

            pose_marker = Marker()
            pose_marker.header.frame_id = frame_id
            pose_marker.header.stamp = now
            pose_marker.ns = 'waypoint_pose'
            pose_marker.id = base_id
            pose_marker.type = Marker.ARROW
            pose_marker.action = Marker.ADD
            pose_marker.pose.position.x = x
            pose_marker.pose.position.y = y
            pose_marker.pose.position.z = z
            pose_marker.pose.orientation.x = qx
            pose_marker.pose.orientation.y = qy
            pose_marker.pose.orientation.z = qz
            pose_marker.pose.orientation.w = qw
            pose_marker.scale.x = 0.35
            pose_marker.scale.y = 0.08
            pose_marker.scale.z = 0.08
            pose_marker.color.r = 0.1
            pose_marker.color.g = 0.8
            pose_marker.color.b = 0.2
            pose_marker.color.a = 1.0
            msg.markers.append(pose_marker)

            text_marker = Marker()
            text_marker.header.frame_id = frame_id
            text_marker.header.stamp = now
            text_marker.ns = 'waypoint_label'
            text_marker.id = base_id + 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = x
            text_marker.pose.position.y = y
            text_marker.pose.position.z = z + self.text_offset_z + 0.02
            text_marker.pose.orientation.w = 1.0
            # Increase font size for better visibility
            text_marker.scale.z = 0.6
            # Primary text color: red
            text_marker.color.r = 1.0
            text_marker.color.g = 0.0
            text_marker.color.b = 0.0
            text_marker.color.a = 1.0
            text_marker.text = name
            # Add a semi-transparent yellow background marker slightly below the text
            bg_marker = Marker()
            bg_marker.header.frame_id = frame_id
            bg_marker.header.stamp = now
            bg_marker.ns = 'waypoint_label_bg'
            bg_marker.id = base_id + 2
            bg_marker.type = Marker.CYLINDER
            bg_marker.action = Marker.ADD
            bg_marker.pose.position.x = x
            bg_marker.pose.position.y = y
            # place the background just below the text to avoid Z-fighting
            bg_marker.pose.position.z = z + self.text_offset_z - 0.02
            bg_marker.pose.orientation.w = 1.0
            # make background a flat disk-like cylinder (smaller radius to avoid covering robot)
            bg_marker.scale.x = 0.3
            bg_marker.scale.y = 0.3
            bg_marker.scale.z = 0.02
            bg_marker.color.r = 1.0
            bg_marker.color.g = 1.0
            bg_marker.color.b = 0.0
            bg_marker.color.a = 0.6

            msg.markers.append(bg_marker)
            msg.markers.append(text_marker)

        return msg


def main(args=None):
    rclpy.init(args=args)
    node = WaypointMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])
