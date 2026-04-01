#!/usr/bin/env python3
import argparse
import json
import math
import time
import select
import sys
import os
from typing import Dict, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class WaypointGoalSender(Node):
    def __init__(self):
        super().__init__('waypoint_goal_sender')

        self.declare_parameter('waypoint_file', 'src/M3Pro_robot_navigation/config/waypoints.json')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('navigate_action', 'navigate_to_pose')

        self.waypoint_file = self.get_parameter('waypoint_file').value
        self.global_frame = self.get_parameter('global_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.navigate_action = self.get_parameter('navigate_action').value

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._nav_client = ActionClient(self, NavigateToPose, self.navigate_action)

    def _resolve_waypoint_file(self) -> str:
        if os.path.isabs(self.waypoint_file):
            return self.waypoint_file
        return os.path.abspath(self.waypoint_file)

    def load_waypoints(self) -> Dict[str, dict]:
        file_path = self._resolve_waypoint_file()
        if not os.path.exists(file_path):
            return {}

        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except (json.JSONDecodeError, OSError):
            return {}

        if not isinstance(data, dict):
            return {}

        waypoints = data.get('waypoints')
        if not isinstance(waypoints, dict):
            return {}

        return waypoints

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _quaternion_from_yaw(yaw: float):
        half = yaw * 0.5
        return 0.0, 0.0, math.sin(half), math.cos(half)

    def get_current_location(self) -> Optional[dict]:
        try:
            trans = self._tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                rclpy.time.Time(),
            )
        except TransformException:
            return None

        t = trans.transform.translation
        q = trans.transform.rotation
        yaw = self._yaw_from_quaternion(q.x, q.y, q.z, q.w)

        return {
            'x': float(t.x),
            'y': float(t.y),
            'z': float(t.z),
            'yaw_rad': float(yaw),
            'yaw_deg': float(math.degrees(yaw)),
        }

    def location_str(self) -> str:
        loc = self.get_current_location()
        if loc is None:
            return 'unknown'
        return f"x={loc['x']:.3f}, y={loc['y']:.3f}, yaw={loc['yaw_deg']:.1f} deg"

    def wait_for_initial_location(self):
        last_print = 0.0
        while rclpy.ok() and self.get_current_location() is None:
            now = time.time()
            # print at most once every 10 seconds
            if now - last_print >= 10.0:
                self.get_logger().info(
                    f'Waiting for TF {self.global_frame} -> {self.base_frame} ...'
                )
                last_print = now
            rclpy.spin_once(self, timeout_sec=0.5)

    def wait_for_nav_server(self):
        while rclpy.ok() and not self._nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for action server: {self.navigate_action}')

    def _build_goal_pose(self, wp: dict) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = wp.get('frame_id', self.global_frame)
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(wp.get('x', 0.0))
        pose.pose.position.y = float(wp.get('y', 0.0))
        pose.pose.position.z = float(wp.get('z', 0.0))

        if all(k in wp for k in ('qx', 'qy', 'qz', 'qw')):
            pose.pose.orientation.x = float(wp['qx'])
            pose.pose.orientation.y = float(wp['qy'])
            pose.pose.orientation.z = float(wp['qz'])
            pose.pose.orientation.w = float(wp['qw'])
        else:
            yaw = float(wp.get('yaw_rad', math.radians(float(wp.get('yaw_deg', 0.0)))))
            qx, qy, qz, qw = self._quaternion_from_yaw(yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

        return pose

    def navigate_to_waypoint(self, wp: dict) -> tuple:
        goal = NavigateToPose.Goal()
        goal.pose = self._build_goal_pose(wp)

        send_future = self._nav_client.send_goal_async(goal)
        while rclpy.ok() and not send_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        if not send_future.done() or send_future.result() is None:
            return False, False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            return False, False

        result_future = goal_handle.get_result_async()
        last_print = time.time()
        quit_requested = False
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.2)
            # print periodic status every 1 second and check for user input
            now = time.time()
            if now - last_print >= 15.0:
                self.get_logger().info('still navigating... (Input "a" to abort navigation, Input "q" to quit)')
                last_print = now
            # non-blocking stdin check
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline().strip()
                if not line:
                    continue
                l = line.lower()
                if l in {'a', 'abort'}:
                    self.get_logger().info('Abort requested by user, cancelling goal...')
                    cancel_future = goal_handle.cancel_goal_async()
                    while rclpy.ok() and not cancel_future.done():
                        rclpy.spin_once(self, timeout_sec=0.1)
                    continue
                if l in {'q', 'quit', 'exit'}:
                    self.get_logger().info('Quit requested by user, cancelling goal and exiting...')
                    quit_requested = True
                    cancel_future = goal_handle.cancel_goal_async()
                    while rclpy.ok() and not cancel_future.done():
                        rclpy.spin_once(self, timeout_sec=0.1)
                    # continue waiting for result to reflect cancelled status
                    continue

        if not result_future.done() or result_future.result() is None:
            return False, quit_requested

        wrapped_result = result_future.result()
        success = wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
        return success, quit_requested


def main(args=None):
    argv = args if args is not None else sys.argv[1:]

    parser = argparse.ArgumentParser(description='Send goal to saved waypoint')
    parser.add_argument('--list', action='store_true', help='List all waypoint names and coordinates')
    parser.add_argument('--echo', metavar='NAME', help='Print one waypoint detail by name')
    parser.add_argument('--name', metavar='NAME', help='Navigate to the specified waypoint name')
    parsed, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = WaypointGoalSender()

    try:
        if parsed.list:
            waypoints = node.load_waypoints()
            if not waypoints:
                print('No waypoints found.')
                return
            for name in sorted(waypoints.keys()):
                wp = waypoints[name]
                print(f"{name}: x={wp.get('x', 0.0):.3f}, y={wp.get('y', 0.0):.3f}, yaw_deg={wp.get('yaw_deg', 0.0):.1f}")
            return

        if parsed.echo:
            waypoints = node.load_waypoints()
            target = waypoints.get(parsed.echo)
            if target is None:
                print('Name doesn`t exist.')
                return
            print(json.dumps(target, indent=2, ensure_ascii=False))
            return

        if parsed.name:
            waypoints = node.load_waypoints()
            target = waypoints.get(parsed.name)
            if target is None:
                print('Name doesn`t exist.')
                return

            node.wait_for_initial_location()
            node.wait_for_nav_server()
            print(f'start nav to {parsed.name} ...')
            success, _ = node.navigate_to_waypoint(target)
            print(f'navigation complete: {"success" if success else "failed"}')
            return

        node.wait_for_initial_location()
        node.wait_for_nav_server()

        while rclpy.ok():
            current = node.location_str()
            user_input = input(
                f'current location ({current}), please enter target name '
                '(Input "q" to quit. Input "l" to list all names): '
            ).strip()

            if not user_input:
                continue

            lowered = user_input.lower()
            if lowered in {'q', 'quit', 'exit'}:
                break

            if lowered == 'l':
                waypoints = node.load_waypoints()
                if not waypoints:
                    print('No waypoints found.')
                else:
                    print('Waypoints:')
                    for name in sorted(waypoints.keys()):
                        wp = waypoints[name]
                        print(f"- {name}: x={wp.get('x', 0.0)}, y={wp.get('y', 0.0)}, yaw_deg={wp.get('yaw_deg', 0.0)}")
                print('')
                continue

            waypoints = node.load_waypoints()
            target = waypoints.get(user_input)
            if target is None:
                print('Name doesn`t exist.')
                print('')
                continue

            print(f'start nav to {user_input} ...')
            success, quit_now = node.navigate_to_waypoint(target)
            print(f'navigation complete: {"success" if success else "failed"}')
            if quit_now:
                break
            print('')

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
