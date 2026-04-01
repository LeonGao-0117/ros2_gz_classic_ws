#!/usr/bin/env python3
import argparse
import math
import shutil
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


def yaw_to_quat(yaw_rad: float):
    half = yaw_rad * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def run_cmd(cmd, timeout_sec=None):
    try:
        return subprocess.run(cmd, text=True, capture_output=True, timeout=timeout_sec)
    except subprocess.TimeoutExpired as e:
        stdout = e.stdout if e.stdout is not None else ''
        stderr = e.stderr if e.stderr is not None else ''
        if timeout_sec is not None:
            stderr = (stderr + f'\nCommand timeout after {timeout_sec:.1f}s').strip()
        return subprocess.CompletedProcess(cmd, returncode=124, stdout=stdout, stderr=stderr)


def set_gazebo_pose(model_name: str, world_x: float, world_y: float, world_z: float, world_yaw_deg: float):
    yaw_rad = math.radians(world_yaw_deg)
    qx, qy, qz, qw = yaw_to_quat(yaw_rad)

    res = run_cmd(['ros2', 'service', 'list'])
    services = res.stdout.splitlines() if res.returncode == 0 else []

    set_service = None
    if '/gazebo/set_entity_state' in services:
        set_service = '/gazebo/set_entity_state'
    elif '/set_entity_state' in services:
        set_service = '/set_entity_state'

    if set_service:
        req = (
            "{state: {name: '%s', pose: {position: {x: %s, y: %s, z: %s}, "
            "orientation: {x: %s, y: %s, z: %s, w: %s}}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, "
            "angular: {x: 0.0, y: 0.0, z: 0.0}}, reference_frame: 'world'}}"
            % (model_name, world_x, world_y, world_z, qx, qy, qz, qw)
        )
        cmd = ['ros2', 'service', 'call', set_service, 'gazebo_msgs/srv/SetEntityState', req]
        res = run_cmd(cmd)
        if res.returncode == 0:
            print(f'Gazebo pose reset via ROS service: {set_service}')
            return True
        print('Failed to call set_entity_state service, fallback to gz model.')
        if res.stderr:
            print(res.stderr.strip())

    gz_bin = shutil.which('gz')
    if gz_bin:
        cmd = [
            gz_bin,
            'model',
            '-m',
            model_name,
            '-x',
            str(world_x),
            '-y',
            str(world_y),
            '-z',
            str(world_z),
            '-R',
            '0',
            '-P',
            '0',
            '-Y',
            str(yaw_rad),
        ]
        res = run_cmd(cmd)
        if res.returncode == 0:
            print('Gazebo pose reset via gz model CLI.')
            return True
        print('Failed to set gazebo pose with gz model.')
        if res.stderr:
            print(res.stderr.strip())
        return False

    print('No /set_entity_state service and no gz executable found.')
    return False


def _clear_one_costmap(
    node: Node,
    service_name: str,
    wait_service_sec: float,
    call_timeout_sec: float,
    retries: int,
    retry_interval_sec: float,
) -> bool:
    client = node.create_client(ClearEntireCostmap, service_name)
    try:
        for attempt in range(1, retries + 1):
            if not client.wait_for_service(timeout_sec=wait_service_sec):
                node.get_logger().warn(
                    f'[{service_name}] not available (attempt {attempt}/{retries}), wait={wait_service_sec:.1f}s'
                )
            else:
                req = ClearEntireCostmap.Request()
                future = client.call_async(req)
                rclpy.spin_until_future_complete(node, future, timeout_sec=call_timeout_sec)

                if future.done() and future.result() is not None:
                    node.get_logger().info(f'[{service_name}] clear succeeded (attempt {attempt}/{retries})')
                    return True

                exc = future.exception() if future.done() else None
                if exc is None:
                    node.get_logger().warn(
                        f'[{service_name}] call timeout (attempt {attempt}/{retries}), timeout={call_timeout_sec:.1f}s'
                    )
                else:
                    node.get_logger().warn(f'[{service_name}] call failed (attempt {attempt}/{retries}): {exc}')

            if attempt < retries:
                time.sleep(retry_interval_sec)

        return False
    finally:
        node.destroy_client(client)


def clear_costmaps(
    node: Node,
    wait_service_sec: float = 1.0,
    call_timeout_sec: float = 2.0,
    retries: int = 3,
    retry_interval_sec: float = 0.5,
):
    """Clear Nav2 global/local costmaps using rclpy service clients with retry/wait."""
    node.get_logger().info('Attempting to clear Nav2 costmaps...')

    # Candidate names cover both common layouts:
    # 1) /global_costmap/clear_entirely_global_costmap
    # 2) /global_costmap/global_costmap/clear_entirely_global_costmap
    global_candidates = [
        '/global_costmap/clear_entirely_global_costmap',
        '/global_costmap/global_costmap/clear_entirely_global_costmap',
    ]
    local_candidates = [
        '/local_costmap/clear_entirely_local_costmap',
        '/local_costmap/local_costmap/clear_entirely_local_costmap',
    ]

    all_services = {name for name, _types in node.get_service_names_and_types()}

    def pick_service(candidates):
        for s in candidates:
            if s in all_services:
                return s
        return candidates[0]

    services = [
        pick_service(global_candidates),
        pick_service(local_candidates),
    ]

    node.get_logger().info(f'Costmap clear services selected: {services}')

    result = {}
    for service_name in services:
        ok = _clear_one_costmap(
            node=node,
            service_name=service_name,
            wait_service_sec=wait_service_sec,
            call_timeout_sec=call_timeout_sec,
            retries=retries,
            retry_interval_sec=retry_interval_sec,
        )
        result[service_name] = ok

    success_count = sum(1 for v in result.values() if v)
    node.get_logger().info(f'Costmap clear summary: {success_count}/{len(services)} services succeeded.')
    return success_count > 0


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('reset_to_init_pose_publisher')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    def wait_for_clock(self, timeout_sec: float):
        self.get_logger().info('Waiting for /clock topic and time synchronization...')
        start_wall = time.time()
        while (time.time() - start_wall) < timeout_sec and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            now = self.get_clock().now()
            # ignore zero/unsynced sim time
            # consider clock synced when time is non-zero (allow small sim times)
            if now.nanoseconds > 0 and now.seconds_nanoseconds()[0] > 0:
                self.get_logger().info(
                    f'Clock synchronized: {now.seconds_nanoseconds()[0]}.{now.seconds_nanoseconds()[1]}'
                )
                return True
            time.sleep(0.1)

        self.get_logger().warn('Clock synchronization timeout. Publishing anyway...')
        return False

    def publish_initialpose(self, map_x: float, map_y: float, map_yaw_deg: float, repeats: int):
        yaw_rad = math.radians(map_yaw_deg)
        qx, qy, qz, qw = yaw_to_quat(yaw_rad)

        for i in range(repeats):
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()

            msg.pose.pose.position.x = map_x
            msg.pose.pose.position.y = map_y
            msg.pose.pose.position.z = 0.0
            msg.pose.pose.orientation.x = qx
            msg.pose.pose.orientation.y = qy
            msg.pose.pose.orientation.z = qz
            msg.pose.pose.orientation.w = qw

            cov = [0.0] * 36
            cov[0] = 0.25
            cov[7] = 0.25
            cov[35] = 0.0685
            msg.pose.covariance = cov

            self.pub.publish(msg)
            self.get_logger().info(
                f'Published /initialpose {i + 1}/{repeats} at x={map_x:.3f}, y={map_y:.3f}, yaw={map_yaw_deg:.1f} deg'
            )
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.5)


def build_parser():
    parser = argparse.ArgumentParser(
        description='Reset robot to initial pose in both Gazebo world and map (/initialpose).'
    )
    parser.add_argument('--model-name', default='M3Pro', help='Gazebo model name')
    # Default world pose is based on hospital startup config:
    # src/M3Pro_robot_bringup/launch/M3Pro_robot.launch.py sets hospital spawn_xyz=[0.0, 3.0, 0.15].
    # This script uses the near-ground settled pose measured from Gazebo after startup
    # (x~-0.0, y=3.0, z~-0.001005) so reset behavior matches runtime state in hospital world.
    parser.add_argument('--world-x', type=float, default=-0.0, help='Gazebo world X')
    parser.add_argument('--world-y', type=float, default=3.0, help='Gazebo world Y')
    parser.add_argument('--world-z', type=float, default=-0.001005, help='Gazebo world Z')
    parser.add_argument('--world-yaw', type=float, default=0.0, help='Gazebo world yaw in degrees')
    parser.add_argument('--map-x', type=float, default=0.0, help='Map X for /initialpose')
    parser.add_argument('--map-y', type=float, default=0.0, help='Map Y for /initialpose')
    parser.add_argument('--map-yaw', type=float, default=0.0, help='Map yaw in degrees for /initialpose')
    parser.add_argument('--clock-timeout', type=float, default=10.0, help='Timeout waiting for /clock')
    parser.add_argument('--repeats', type=int, default=3, help='How many times to publish /initialpose')
    parser.add_argument('--amcl-wait', type=float, default=10.0, help='Seconds to wait for /amcl_pose after publishing')
    parser.add_argument('--costmap-retries', type=int, default=3, help='Retry count for each costmap clear service call')
    parser.add_argument('--costmap-wait-service', type=float, default=1.0, help='Wait seconds for service availability per attempt')
    parser.add_argument('--costmap-call-timeout', type=float, default=2.0, help='Timeout seconds per service call attempt')
    parser.add_argument('--costmap-retry-interval', type=float, default=0.5, help='Sleep seconds between retry attempts')
    parser.add_argument('--cmd-timeout', type=float, default=5.0, help='Timeout seconds for external CLI commands (gz/ros2 topic echo)')
    return parser


def main(args=None):
    parser = build_parser()
    parsed, ros_args = parser.parse_known_args(args=args)

    print('=== reset_to_init_pose START ===')
    print(f'model_name={parsed.model_name}')
    print(
        f'WORLD=({parsed.world_x}, {parsed.world_y}, {parsed.world_z}, yaw_deg={parsed.world_yaw})'
    )
    print(f'MAP=({parsed.map_x}, {parsed.map_y}, yaw_deg={parsed.map_yaw})')

    world_ok = set_gazebo_pose(
        parsed.model_name,
        parsed.world_x,
        parsed.world_y,
        parsed.world_z,
        parsed.world_yaw,
    )

    rclpy.init(args=ros_args)
    node = InitialPosePublisher()

    try:
        node.wait_for_clock(parsed.clock_timeout)
        node.publish_initialpose(parsed.map_x, parsed.map_y, parsed.map_yaw, parsed.repeats)
        # attempt to clear Nav2 costmaps (global and local) so stale obstacles are removed
        print('Clearing costmaps...')
        try:
            clear_costmaps(
                node,
                wait_service_sec=parsed.costmap_wait_service,
                call_timeout_sec=parsed.costmap_call_timeout,
                retries=max(1, parsed.costmap_retries),
                retry_interval_sec=max(0.0, parsed.costmap_retry_interval),
            )
        except Exception as e:
            print(f'Failed to clear costmaps: {e}')
        # attempt to read current AMCL (/amcl_pose) once
        try:
            # subscribe once to /amcl_pose to get robot pose in map
            from geometry_msgs.msg import PoseWithCovarianceStamped

            amcl_msg = None

            def _amcl_cb(msg):
                nonlocal amcl_msg
                amcl_msg = msg

            # Create subscription with QoS matching the /amcl_pose publisher.
            # /amcl_pose is published with Durability=TRANSIENT_LOCAL and RELIABLE reliability,
            # so subscribers must use the same DurabilityPolicy to receive the latched (cached)
            # message immediately upon subscribing. Using default QoS may miss the cached sample.
            qos = QoSProfile(depth=1)
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            qos.reliability = ReliabilityPolicy.RELIABLE
            sub = node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', _amcl_cb, qos_profile=qos)
            start_t = time.time()
            # only wait 1s for in-process subscription (use ros2 topic echo fallback if needed)
            amcl_wait = 1.0
            while rclpy.ok() and amcl_msg is None and (time.time() - start_t) < amcl_wait:
                rclpy.spin_once(node, timeout_sec=0.1)
            try:
                node.destroy_subscription(sub)
            except Exception:
                pass

            if amcl_msg is not None:
                px = amcl_msg.pose.pose.position.x
                py = amcl_msg.pose.pose.position.y
                q = amcl_msg.pose.pose.orientation
                yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
                print(f"Map (AMCL) pose: x={px:.3f}, y={py:.3f}, yaw={math.degrees(yaw):.1f} deg")
            else:
                print('amcl_msg is None, try using `ros2 topic echo /amcl_pose --once` to retrieve a sample')
                # fallback: try using `ros2 topic echo /amcl_pose --once` to retrieve a sample
                try:
                    echo_res = run_cmd(['ros2', 'topic', 'echo', '/amcl_pose', '--once'], timeout_sec=parsed.cmd_timeout)
                    if echo_res.returncode == 0 and echo_res.stdout:
                        out = echo_res.stdout
                        # crude parsing for position and orientation
                        px = py = None
                        qx = qy = qz = qw = None
                        for line in out.splitlines():
                            line = line.strip()
                            if line.startswith('position:'):
                                # next lines contain x/y/z
                                continue
                            if line.startswith('x:') and px is None:
                                try:
                                    px = float(line.split(':', 1)[1].strip())
                                except Exception:
                                    pass
                            if line.startswith('y:') and py is None:
                                try:
                                    py = float(line.split(':', 1)[1].strip())
                                except Exception:
                                    pass
                            if line.startswith('orientation:'):
                                continue
                            if line.startswith('x:') and qx is None and ('orientation' in out):
                                try:
                                    # ambiguous x: lines; rely on order after 'orientation:'
                                    qx = float(line.split(':', 1)[1].strip())
                                except Exception:
                                    pass
                            if line.startswith('y:') and qy is None and ('orientation' in out):
                                try:
                                    qy = float(line.split(':', 1)[1].strip())
                                except Exception:
                                    pass
                            if line.startswith('z:') and qz is None and ('orientation' in out):
                                try:
                                    qz = float(line.split(':', 1)[1].strip())
                                except Exception:
                                    pass
                            if line.startswith('w:') and qw is None and ('orientation' in out):
                                try:
                                    qw = float(line.split(':', 1)[1].strip())
                                except Exception:
                                    pass

                        if px is not None and py is not None:
                            if qx is not None and qy is not None and qz is not None and qw is not None:
                                yaw = quat_to_yaw(qx, qy, qz, qw)
                                print(f"Map (AMCL) pose (from ros2 topic echo): x={px:.3f}, y={py:.3f}, yaw={math.degrees(yaw):.1f} deg")
                            else:
                                print(f"Map (AMCL) pose (from ros2 topic echo): x={px:.3f}, y={py:.3f}")
                        else:
                            print('Map (AMCL) pose: not available')
                    else:
                        print('Map (AMCL) pose: not available')
                except Exception:
                    print('Map (AMCL) pose: not available')
        except Exception as e:
            print(f'Failed to read /amcl_pose: {e}')

        # read Gazebo model pose via gz model -p
        try:
            gz_res = run_cmd(['gz', 'model', '-m', parsed.model_name, '-p'], timeout_sec=parsed.cmd_timeout)
            if gz_res.returncode == 0 and gz_res.stdout:
                raw = gz_res.stdout.strip()
                print(f'Gazebo model pose (raw): {raw}')
                # try to parse numbers: x y z [roll pitch yaw]
                parts = raw.split()
                nums = []
                for p in parts:
                    try:
                        nums.append(float(p))
                    except Exception:
                        pass

                if len(nums) >= 3:
                    gx, gy, gz_z = nums[0], nums[1], nums[2]
                    yaw_deg = None
                    if len(nums) >= 6:
                        # assume roll pitch yaw in radians at positions 3,4,5
                        yaw_rad = nums[5]
                        yaw_deg = math.degrees(yaw_rad)

                    if yaw_deg is None:
                        print(f'Gazebo world pose: x={gx:.3f}, y={gy:.3f}, z={gz_z:.3f}')
                    else:
                        print(f'Gazebo world pose: x={gx:.3f}, y={gy:.3f}, z={gz_z:.3f}, yaw={yaw_deg:.1f} deg')
                else:
                    print('Could not parse gazebo model pose numbers')
            else:
                print('Gazebo model pose: could not retrieve (gz returned non-zero)')
                if gz_res.stderr:
                    print(gz_res.stderr.strip())
        except Exception as e:
            print(f'Failed to query gz model pose: {e}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if world_ok:
        print('Reset complete: gazebo world + map initialpose done.')
    else:
        print('Partial complete: map initialpose done, gazebo world reset may have failed.')
    print('=== reset_to_init_pose END ===')


if __name__ == '__main__':
    main(sys.argv[1:])
