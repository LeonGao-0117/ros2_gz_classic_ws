#!/usr/bin/env python3
import argparse
import math
import os
import select
import sys
from datetime import datetime

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
UTIL_PATHS = [
    os.path.join(SCRIPT_DIR, 'utils'),
    os.path.join(SCRIPT_DIR, 'scripts', 'utils'),
]
for util_path in UTIL_PATHS:
    if os.path.isdir(util_path) and util_path not in sys.path:
        sys.path.append(util_path)

from waypoint_io import load_waypoints as io_load_waypoints, write_waypoints as io_write_waypoints


class WaypointCollector(Node):
    def __init__(self):
        super().__init__('waypoint_collector')

        self.declare_parameter('pose_topic', '/initialpose')
        self.declare_parameter(
            'output_file',
            'src/M3Pro_robot_navigation/config/waypoints.json',
        )

        self.pose_topic = self.get_parameter('pose_topic').value
        self.output_file = self.get_parameter('output_file').value

        self._latest_pose = None

        self.create_subscription(
            PoseWithCovarianceStamped,
            self.pose_topic,
            self._pose_callback,
            10,
        )

        self.get_logger().info(
            f'Listening pose from: {self.pose_topic} | output: {self.output_file}'
        )

    def _pose_callback(self, msg: PoseWithCovarianceStamped):
        pose = msg.pose.pose
        q = pose.orientation
        yaw = self._yaw_from_quaternion(q.x, q.y, q.z, q.w)

        pose_data = {
            'frame_id': msg.header.frame_id,
            'x': float(pose.position.x),
            'y': float(pose.position.y),
            'z': float(pose.position.z),
            'yaw_rad': float(yaw),
            'yaw_deg': float(math.degrees(yaw)),
            'qx': float(q.x),
            'qy': float(q.y),
            'qz': float(q.z),
            'qw': float(q.w),
            'ros_stamp_sec': int(msg.header.stamp.sec),
            'ros_stamp_nanosec': int(msg.header.stamp.nanosec),
            'saved_at': datetime.now().isoformat(timespec='seconds'),
        }

        self._latest_pose = pose_data

        self.get_logger().info(
            f"Pose updated: x={pose_data['x']:.3f}, y={pose_data['y']:.3f}, yaw={pose_data['yaw_deg']:.1f} deg"
        )

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def get_latest_pose(self):
        if self._latest_pose is None:
            return None
        return dict(self._latest_pose)

    def clear_latest_pose(self):
        self._latest_pose = None

    def waypoint_exists(self, name: str) -> bool:
        loaded = self.load_all()
        waypoints = loaded.get('waypoints', {})
        return isinstance(waypoints, dict) and name in waypoints

    def load_all(self):
        return io_load_waypoints(self.output_file)

    def write_all(self, data: dict):
        io_write_waypoints(self.output_file, data)

    def save_waypoint(self, name: str, pose_data=None) -> bool:
        target_pose = dict(pose_data) if pose_data is not None else self.get_latest_pose()
        if target_pose is None:
            self.get_logger().warning('No pose received yet, cannot save waypoint.')
            return False

        data = self.load_all()
        data['waypoints'][name] = target_pose
        self.write_all(data)

        self.get_logger().info(
            f"Saved waypoint '{name}' -> x={target_pose['x']:.3f}, y={target_pose['y']:.3f}, yaw={target_pose['yaw_deg']:.1f} deg"
        )
        return True


def main(args=None):
    parser = argparse.ArgumentParser(description='Waypoint collector / manager')
    parser.add_argument('--list', action='store_true', help='List all saved waypoints')
    parser.add_argument('--echo', metavar='NAME', help='Print details of a waypoint')
    parser.add_argument('--remove', metavar='NAME', help='Remove a waypoint by name')
    parser.add_argument(
        '--add', nargs=3, metavar=('NAME', 'X', 'Y'), help='Add a waypoint with NAME X Y (yaw in degrees optional with --yaw)'
    )
    parser.add_argument('--yaw', type=float, default=0.0, help='Yaw in degrees for --add')
    parser.add_argument('--z', type=float, default=0.0, help='Z coordinate for --add')
    parser.add_argument('--frame', default='map', help='Frame id for --add')

    parsed = parser.parse_args(args=args)

    # If any manager action is requested, operate on file without spinning ROS
    if parsed.list or parsed.echo or parsed.remove or parsed.add:
        # make a temporary node to get default output_file path
        rclpy.init(args=args)
        node = WaypointCollector()
        try:
            data = node.load_all()

            if parsed.list:
                waypoints = data.get('waypoints', {})
                if not waypoints:
                    print('No waypoints saved.')
                else:
                    for name, p in waypoints.items():
                        print(f"{name}: x={p['x']:.3f}, y={p['y']:.3f}, yaw_deg={p['yaw_deg']:.1f}")
                return

            if parsed.echo:
                w = data.get('waypoints', {}).get(parsed.echo)
                if not w:
                    print(f'Waypoint "{parsed.echo}" not found.')
                else:
                    print(json.dumps(w, indent=2, ensure_ascii=False))
                return

            if parsed.remove:
                if parsed.remove in data.get('waypoints', {}):
                    del data['waypoints'][parsed.remove]
                    node.write_all(data)
                    print(f'Removed waypoint "{parsed.remove}"')
                else:
                    print(f'Waypoint "{parsed.remove}" not found.')
                return

            if parsed.add:
                name, xs, ys = parsed.add
                try:
                    x = float(xs)
                    y = float(ys)
                except ValueError:
                    print('Invalid X/Y values for --add')
                    return

                if parsed.frame:
                    frame_id = parsed.frame
                else:
                    frame_id = 'map'

                new_pose = {
                    'frame_id': frame_id,
                    'x': float(x),
                    'y': float(y),
                    'z': float(parsed.z),
                    'yaw_rad': float(math.radians(parsed.yaw)),
                    'yaw_deg': float(parsed.yaw),
                    'qx': 0.0,
                    'qy': 0.0,
                    'qz': 0.0,
                    'qw': 1.0,
                    'ros_stamp_sec': 0,
                    'ros_stamp_nanosec': 0,
                    'saved_at': datetime.now().isoformat(timespec='seconds'),
                }

                if name in data.get('waypoints', {}):
                    while True:
                        overwrite = input(f'Waypoint "{name}" already exists. Overwrite? (y/N): ').strip().lower()
                        if overwrite in {'y', 'yes'}:
                            break
                        if overwrite in {'', 'n', 'no'}:
                            print('Canceled. Not adding waypoint.')
                            return
                        print('Invalid input. Please input y or n.')

                data.setdefault('waypoints', {})[name] = new_pose
                node.write_all(data)
                print(f'Added waypoint "{name}"')
                return
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    # Otherwise run interactive collector
    rclpy.init(args=args)
    node = WaypointCollector()

    print('\n=== Waypoint Collector ===')
    print('Use RViz2 "2D Pose Estimate" to publish pose first.')
    print('Each received pose will pause for your decision.')
    print('Input "i" to ignore current pose, "q" to quit.\n')

    try:
        while rclpy.ok():
            node.clear_latest_pose()
            print(f'\n==== Waiting for topic {node.pose_topic} (Input "q" to quit): ====')

            # wait for either topic or user typing 'q' + Enter
            while rclpy.ok() and node.get_latest_pose() is None:
                rclpy.spin_once(node, timeout_sec=0.5)
                # check stdin for immediate quit
                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    line = sys.stdin.readline().strip()
                    if line.lower() in {'q', 'quit', 'exit'}:
                        print(f'Waypoints saved in: {os.path.abspath(node.output_file)}')
                        return
                    else:
                        # ignore other inputs while waiting
                        print('Input ignored while waiting for topic. (Type "q" to quit)')

            if not rclpy.ok():
                break

            current_pose = node.get_latest_pose()
            if current_pose is None:
                continue

            while rclpy.ok():
                user_input = input(
                    'please input waypoint name (Input "i" to ignore. Input "q" to quit): '
                ).strip()

                if not user_input:
                    print('Name cannot be empty.')
                    continue

                lowered = user_input.lower()
                if lowered in {'q', 'quit', 'exit'}:
                    print(f'Waypoints saved in: {os.path.abspath(node.output_file)}')
                    return

                if lowered == 'i':
                    print('Ignored current pose.')
                    break

                if node.waypoint_exists(user_input):
                    while True:
                        overwrite = input(
                            f'Waypoint "{user_input}" already exists. Overwrite? (y/N): '
                        ).strip().lower()
                        if overwrite in {'y', 'yes'}:
                            break
                        if overwrite in {'', 'n', 'no'}:
                            print('Canceled overwrite. Please input another waypoint name.')
                            user_input = ''
                            break
                        print('Invalid input. Please input y or n.')

                    if not user_input:
                        continue

                node.save_waypoint(user_input, current_pose)
                break
    except KeyboardInterrupt:
        print(f'\nWaypoints saved in: {os.path.abspath(node.output_file)}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
