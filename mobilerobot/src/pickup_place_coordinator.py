#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PickupPlaceCoordinator(Node):
    """
    Chọn payload_active_path theo phase hiện tại.
    """

    def __init__(self) -> None:
        super().__init__('pickup_place_coordinator')

        self.declare_parameter('team_name', 'team_1')
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('hold_distance', 0.05)
        self.declare_parameter('verbose', True)

        self.team_name = str(self.get_parameter('team_name').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.hold_distance = float(self.get_parameter('hold_distance').value)
        self.verbose = bool(self.get_parameter('verbose').value)

        self.team_state: str = 'IDLE'
        self.payload_global_path: Optional[Path] = None
        self.pickup_pose: Optional[PoseStamped] = None
        self.dropoff_pose: Optional[PoseStamped] = None

        self.state_sub = self.create_subscription(
            String,
            f'/{self.team_name}/task_state',
            self.team_state_callback,
            10
        )

        self.payload_path_sub = self.create_subscription(
            Path,
            f'/{self.team_name}/payload_global_path',
            self.payload_path_callback,
            10
        )

        self.pickup_sub = self.create_subscription(
            PoseStamped,
            f'/{self.team_name}/pickup_pose',
            self.pickup_callback,
            10
        )

        self.dropoff_sub = self.create_subscription(
            PoseStamped,
            f'/{self.team_name}/dropoff_pose',
            self.dropoff_callback,
            10
        )

        self.operation_mode_pub = self.create_publisher(
            String,
            f'/{self.team_name}/operation_mode',
            10
        )

        self.payload_active_path_pub = self.create_publisher(
            Path,
            f'/{self.team_name}/payload_active_path',
            10
        )

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('Pickup/place coordinator started')

    def team_state_callback(self, msg: String) -> None:
        self.team_state = msg.data

    def payload_path_callback(self, msg: Path) -> None:
        self.payload_global_path = msg

    def pickup_callback(self, msg: PoseStamped) -> None:
        self.pickup_pose = msg

    def dropoff_callback(self, msg: PoseStamped) -> None:
        self.dropoff_pose = msg

    def publish_mode(self, mode: str) -> None:
        msg = String()
        msg.data = mode
        self.operation_mode_pub.publish(msg)

    def build_hold_path_from_pose(self, pose_msg: PoseStamped) -> Path:
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = pose_msg.header.frame_id if pose_msg.header.frame_id else 'odom'

        q = pose_msg.pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

        p1 = PoseStamped()
        p1.header = path.header
        p1.pose.position.x = pose_msg.pose.position.x
        p1.pose.position.y = pose_msg.pose.position.y
        p1.pose.position.z = 0.0
        p1.pose.orientation = quaternion_from_yaw(yaw)

        p2 = PoseStamped()
        p2.header = path.header
        p2.pose.position.x = pose_msg.pose.position.x + self.hold_distance * math.cos(yaw)
        p2.pose.position.y = pose_msg.pose.position.y + self.hold_distance * math.sin(yaw)
        p2.pose.position.z = 0.0
        p2.pose.orientation = quaternion_from_yaw(yaw)

        path.poses = [p1, p2]
        return path

    def timer_callback(self) -> None:
        if self.team_state in ['IDLE', 'DONE']:
            self.publish_mode('HOLD')
            if self.pickup_pose is not None:
                hold_path = self.build_hold_path_from_pose(self.pickup_pose)
                self.payload_active_path_pub.publish(hold_path)
            return

        if self.team_state in ['FORMING', 'GO_TO_PICKUP']:
            self.publish_mode('GO_TO_PICKUP')
            if self.payload_global_path is not None:
                self.payload_active_path_pub.publish(self.payload_global_path)
            return

        if self.team_state in ['PRE_GRASP', 'GRASP_READY']:
            self.publish_mode('HOLD')
            if self.pickup_pose is not None:
                hold_path = self.build_hold_path_from_pose(self.pickup_pose)
                self.payload_active_path_pub.publish(hold_path)
            return

        if self.team_state == 'TRANSPORTING':
            self.publish_mode('COOPERATIVE_TRANSPORT')
            if self.payload_global_path is not None:
                self.payload_active_path_pub.publish(self.payload_global_path)
            return

        if self.team_state in ['PRE_PLACE', 'RELEASING']:
            self.publish_mode('PRE_PLACE')
            if self.dropoff_pose is not None:
                hold_path = self.build_hold_path_from_pose(self.dropoff_pose)
                self.payload_active_path_pub.publish(hold_path)
            return

        self.publish_mode('HOLD')
        if self.pickup_pose is not None:
            hold_path = self.build_hold_path_from_pose(self.pickup_pose)
            self.payload_active_path_pub.publish(hold_path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PickupPlaceCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()