#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
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


class PayloadStateEstimator(Node):
    """
    Ở tầng 2 vẫn dùng 2 carrier active.
    """

    def __init__(self) -> None:
        super().__init__('payload_state_estimator')

        self.declare_parameter('team_name', 'team_1')
        self.declare_parameter('carrier_1_name', 'robot_1')
        self.declare_parameter('carrier_2_name', 'robot_2')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('verbose', True)
        self.declare_parameter('forward_axis_sign', 1.0)

        self.team_name = str(self.get_parameter('team_name').value)
        self.carrier_1_name = str(self.get_parameter('carrier_1_name').value)
        self.carrier_2_name = str(self.get_parameter('carrier_2_name').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.verbose = bool(self.get_parameter('verbose').value)
        self.forward_axis_sign = float(self.get_parameter('forward_axis_sign').value)

        self.carrier_1_odom_topic = f'/{self.carrier_1_name}/odom'
        self.carrier_2_odom_topic = f'/{self.carrier_2_name}/odom'

        self.r1_x: Optional[float] = None
        self.r1_y: Optional[float] = None
        self.r1_yaw: Optional[float] = None

        self.r2_x: Optional[float] = None
        self.r2_y: Optional[float] = None
        self.r2_yaw: Optional[float] = None

        self.r1_sub = self.create_subscription(
            Odometry,
            self.carrier_1_odom_topic,
            self.robot_1_odom_callback,
            10
        )

        self.r2_sub = self.create_subscription(
            Odometry,
            self.carrier_2_odom_topic,
            self.robot_2_odom_callback,
            10
        )

        self.payload_pose_pub = self.create_publisher(
            PoseStamped,
            f'/{self.team_name}/payload_pose',
            10
        )

        self.debug_pub = self.create_publisher(
            String,
            f'/{self.team_name}/payload_state_debug',
            10
        )

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('Payload state estimator started')

    def robot_1_odom_callback(self, msg: Odometry) -> None:
        self.r1_x = msg.pose.pose.position.x
        self.r1_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.r1_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def robot_2_odom_callback(self, msg: Odometry) -> None:
        self.r2_x = msg.pose.pose.position.x
        self.r2_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.r2_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def timer_callback(self) -> None:
        if None in [self.r1_x, self.r1_y, self.r1_yaw, self.r2_x, self.r2_y, self.r2_yaw]:
            return

        px = 0.5 * (self.r1_x + self.r2_x)
        py = 0.5 * (self.r1_y + self.r2_y)

        lateral_yaw = math.atan2(self.r2_y - self.r1_y, self.r2_x - self.r1_x)
        payload_yaw = lateral_yaw + self.forward_axis_sign * (math.pi / 2.0)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'odom'
        pose_msg.pose.position.x = px
        pose_msg.pose.position.y = py
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation = quaternion_from_yaw(payload_yaw)

        self.payload_pose_pub.publish(pose_msg)

        dbg = String()
        dbg.data = (
            f'payload=({px:.3f},{py:.3f},{payload_yaw:.3f}); '
            f'{self.carrier_1_name}=({self.r1_x:.3f},{self.r1_y:.3f},{self.r1_yaw:.3f}); '
            f'{self.carrier_2_name}=({self.r2_x:.3f},{self.r2_y:.3f},{self.r2_yaw:.3f})'
        )
        self.debug_pub.publish(dbg)

        if self.verbose:
            self.get_logger().debug(dbg.data)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PayloadStateEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()