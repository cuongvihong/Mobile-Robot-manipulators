#!/usr/bin/env python3
import math
from typing import List, Optional

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class LocalPathFollower(Node):
    """
    Node generic cho từng robot.
    """

    def __init__(self) -> None:
        super().__init__('local_path_follower')

        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('path_topic', 'segmented_local_path')
        self.declare_parameter('reference_topic', 'tracking_reference')

        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('lookahead_distance', 0.30)
        self.declare_parameter('goal_tolerance', 0.05)

        odom_topic = self.get_parameter('odom_topic').value
        path_topic = self.get_parameter('path_topic').value
        reference_topic = self.get_parameter('reference_topic').value

        self.control_rate = float(self.get_parameter('control_rate').value)
        self.lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)

        self.current_x: Optional[float] = None
        self.current_y: Optional[float] = None
        self.current_yaw: Optional[float] = None

        self.path_msg: Optional[Path] = None

        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        self.path_sub = self.create_subscription(
            Path,
            path_topic,
            self.path_callback,
            10
        )

        self.ref_pub = self.create_publisher(
            PoseStamped,
            reference_topic,
            10
        )

        timer_period = 1.0 / self.control_rate
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info('Local path follower started')

    def odom_callback(self, msg: Odometry) -> None:
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def path_callback(self, msg: Path) -> None:
        self.path_msg = msg

    @staticmethod
    def distance(x1: float, y1: float, x2: float, y2: float) -> float:
        return math.hypot(x2 - x1, y2 - y1)

    def find_closest_index(self, poses: List[PoseStamped], x: float, y: float) -> int:
        best_idx = 0
        best_dist = float('inf')

        for i, pose_stamped in enumerate(poses):
            px = pose_stamped.pose.position.x
            py = pose_stamped.pose.position.y
            d = self.distance(x, y, px, py)
            if d < best_dist:
                best_dist = d
                best_idx = i

        return best_idx

    def find_lookahead_index(self, poses: List[PoseStamped], start_idx: int, x: float, y: float) -> int:
        for i in range(start_idx, len(poses)):
            px = poses[i].pose.position.x
            py = poses[i].pose.position.y
            d = self.distance(x, y, px, py)
            if d >= self.lookahead_distance:
                return i
        return len(poses) - 1

    def control_loop(self) -> None:
        if self.current_x is None or self.current_y is None or self.current_yaw is None:
            return

        if self.path_msg is None or len(self.path_msg.poses) == 0:
            return

        poses = self.path_msg.poses

        goal_pose = poses[-1]
        gx = goal_pose.pose.position.x
        gy = goal_pose.pose.position.y
        dist_to_goal = self.distance(self.current_x, self.current_y, gx, gy)

        if dist_to_goal < self.goal_tolerance:
            ref_msg = PoseStamped()
            ref_msg.header.stamp = self.get_clock().now().to_msg()
            ref_msg.header.frame_id = self.path_msg.header.frame_id if self.path_msg.header.frame_id else 'odom'
            ref_msg.pose = goal_pose.pose
            self.ref_pub.publish(ref_msg)
            return

        closest_idx = self.find_closest_index(poses, self.current_x, self.current_y)
        target_idx = self.find_lookahead_index(poses, closest_idx, self.current_x, self.current_y)
        target_pose = poses[target_idx]

        ref_msg = PoseStamped()
        ref_msg.header.stamp = self.get_clock().now().to_msg()
        ref_msg.header.frame_id = self.path_msg.header.frame_id if self.path_msg.header.frame_id else 'odom'
        ref_msg.pose = target_pose.pose

        self.ref_pub.publish(ref_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalPathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()