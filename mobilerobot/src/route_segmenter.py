#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class RouteSegmenter(Node):
    """
    Node generic cho từng robot.
    Chạy dưới namespace hoặc remap topic riêng.
    """

    def __init__(self) -> None:
        super().__init__('route_segmenter')

        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('global_path_topic', 'local_path')
        self.declare_parameter('local_path_topic', 'segmented_local_path')

        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('backward_points', 1)
        self.declare_parameter('segment_length', 1.5)
        self.declare_parameter('min_points', 3)
        self.declare_parameter('max_points', 30)
        self.declare_parameter('verbose', True)

        odom_topic = self.get_parameter('odom_topic').value
        global_path_topic = self.get_parameter('global_path_topic').value
        local_path_topic = self.get_parameter('local_path_topic').value

        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.backward_points = int(self.get_parameter('backward_points').value)
        self.segment_length = float(self.get_parameter('segment_length').value)
        self.min_points = int(self.get_parameter('min_points').value)
        self.max_points = int(self.get_parameter('max_points').value)
        self.verbose = bool(self.get_parameter('verbose').value)

        self.current_x: Optional[float] = None
        self.current_y: Optional[float] = None
        self.current_yaw: Optional[float] = None

        self.global_path_msg: Optional[Path] = None
        self.last_closest_idx: int = 0

        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        self.global_path_sub = self.create_subscription(
            Path,
            global_path_topic,
            self.global_path_callback,
            10
        )

        self.local_path_pub = self.create_publisher(
            Path,
            local_path_topic,
            10
        )

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('Route segmenter started')

    def odom_callback(self, msg: Odometry) -> None:
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def global_path_callback(self, msg: Path) -> None:
        self.global_path_msg = msg
        self.last_closest_idx = 0

        if self.verbose:
            self.get_logger().info(f'Received global path with {len(msg.poses)} poses')

    @staticmethod
    def distance_xy(x1: float, y1: float, x2: float, y2: float) -> float:
        return math.hypot(x2 - x1, y2 - y1)

    def find_closest_forward_index(self, path: Path, x: float, y: float) -> int:
        poses = path.poses
        n = len(poses)

        if n == 0:
            return 0

        start_idx = max(0, min(self.last_closest_idx, n - 1))

        best_idx = start_idx
        best_dist = float('inf')

        for i in range(start_idx, n):
            px = poses[i].pose.position.x
            py = poses[i].pose.position.y
            d = self.distance_xy(x, y, px, py)
            if d < best_dist:
                best_dist = d
                best_idx = i

        return best_idx

    def build_local_path(self, global_path: Path, start_idx: int) -> Path:
        poses = global_path.poses
        n = len(poses)

        local_path = Path()
        local_path.header.stamp = self.get_clock().now().to_msg()
        local_path.header.frame_id = global_path.header.frame_id if global_path.header.frame_id else 'odom'

        if n == 0:
            return local_path

        seg_start = max(0, start_idx - self.backward_points)

        local_poses = []
        accumulated = 0.0

        prev_x = poses[seg_start].pose.position.x
        prev_y = poses[seg_start].pose.position.y

        local_poses.append(poses[seg_start])

        for i in range(seg_start + 1, n):
            px = poses[i].pose.position.x
            py = poses[i].pose.position.y

            ds = self.distance_xy(prev_x, prev_y, px, py)
            accumulated += ds

            local_poses.append(poses[i])

            prev_x = px
            prev_y = py

            if len(local_poses) >= self.max_points:
                break

            if accumulated >= self.segment_length and len(local_poses) >= self.min_points:
                break

        if len(local_poses) < self.min_points:
            end_idx = min(n, seg_start + self.min_points)
            local_poses = poses[seg_start:end_idx]

        local_path.poses = local_poses
        return local_path

    def timer_callback(self) -> None:
        if self.current_x is None or self.current_y is None or self.current_yaw is None:
            return

        if self.global_path_msg is None or len(self.global_path_msg.poses) == 0:
            return

        closest_idx = self.find_closest_forward_index(
            self.global_path_msg,
            self.current_x,
            self.current_y
        )

        if closest_idx < self.last_closest_idx:
            closest_idx = self.last_closest_idx

        self.last_closest_idx = closest_idx

        local_path = self.build_local_path(self.global_path_msg, closest_idx)
        self.local_path_pub.publish(local_path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RouteSegmenter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()