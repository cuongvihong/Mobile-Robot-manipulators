#!/usr/bin/env python3
import math
from typing import List

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import String


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class FormationGenerator(Node):
    """
    Tầng 2: khóa pair carrier bằng parameter carrier_left_name / carrier_right_name.
    """

    def __init__(self) -> None:
        super().__init__('formation_generator')

        self.declare_parameter('team_name', 'team_1')
        self.declare_parameter('carrier_left_name', 'robot_1')
        self.declare_parameter('carrier_right_name', 'robot_2')

        self.declare_parameter('lateral_offset', 0.40)
        self.declare_parameter('longitudinal_offset', 0.0)

        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('verbose', True)

        self.team_name = str(self.get_parameter('team_name').value)
        self.carrier_left_name = str(self.get_parameter('carrier_left_name').value)
        self.carrier_right_name = str(self.get_parameter('carrier_right_name').value)

        self.lateral_offset = float(self.get_parameter('lateral_offset').value)
        self.longitudinal_offset = float(self.get_parameter('longitudinal_offset').value)

        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.verbose = bool(self.get_parameter('verbose').value)

        self.payload_path: Path = Path()
        self.has_payload_path = False
        self.operation_mode = 'HOLD'

        self.payload_sub = self.create_subscription(
            Path,
            f'/{self.team_name}/payload_active_path',
            self.payload_path_callback,
            10
        )

        self.mode_sub = self.create_subscription(
            String,
            f'/{self.team_name}/operation_mode',
            self.operation_mode_callback,
            10
        )

        self.left_path_pub = self.create_publisher(
            Path,
            f'/{self.carrier_left_name}/local_path',
            10
        )

        self.right_path_pub = self.create_publisher(
            Path,
            f'/{self.carrier_right_name}/local_path',
            10
        )

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('Formation generator started')

    def payload_path_callback(self, msg: Path) -> None:
        self.payload_path = msg
        self.has_payload_path = len(msg.poses) > 0

        if self.verbose:
            self.get_logger().info(f'Received payload active path with {len(msg.poses)} poses')

    def operation_mode_callback(self, msg: String) -> None:
        self.operation_mode = msg.data

    def transform_pose_with_offset(
        self,
        pose_stamped: PoseStamped,
        dx_body: float,
        dy_body: float
    ) -> PoseStamped:
        x = pose_stamped.pose.position.x
        y = pose_stamped.pose.position.y
        q = pose_stamped.pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

        x_out = x + dx_body * math.cos(yaw) - dy_body * math.sin(yaw)
        y_out = y + dx_body * math.sin(yaw) + dy_body * math.cos(yaw)

        out = PoseStamped()
        out.header = pose_stamped.header
        out.pose.position.x = x_out
        out.pose.position.y = y_out
        out.pose.position.z = 0.0
        out.pose.orientation = quaternion_from_yaw(yaw)
        return out

    def build_robot_path(
        self,
        payload_path: Path,
        dx_body: float,
        dy_body: float
    ) -> Path:
        out = Path()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = payload_path.header.frame_id if payload_path.header.frame_id else 'odom'

        out_poses: List[PoseStamped] = []
        for ps in payload_path.poses:
            out_poses.append(self.transform_pose_with_offset(ps, dx_body, dy_body))

        out.poses = out_poses
        return out

    def timer_callback(self) -> None:
        if not self.has_payload_path:
            return

        if self.operation_mode not in ['HOLD', 'GO_TO_PICKUP', 'COOPERATIVE_TRANSPORT', 'PRE_PLACE']:
            return

        left_path = self.build_robot_path(
            self.payload_path,
            self.longitudinal_offset,
            self.lateral_offset
        )

        right_path = self.build_robot_path(
            self.payload_path,
            self.longitudinal_offset,
            -self.lateral_offset
        )

        self.left_path_pub.publish(left_path)
        self.right_path_pub.publish(right_path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FormationGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()