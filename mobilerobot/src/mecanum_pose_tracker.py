#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_to_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class MecanumPoseTracker(Node):
    """
    Node generic cho từng robot.
    """

    def __init__(self) -> None:
        super().__init__('mecanum_pose_tracker')

        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('reference_topic', 'tracking_reference')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')

        self.declare_parameter('control_rate', 30.0)
        self.declare_parameter('k_x', 1.2)
        self.declare_parameter('k_y', 1.2)
        self.declare_parameter('k_yaw', 2.0)

        self.declare_parameter('max_vx', 0.5)
        self.declare_parameter('max_vy', 0.5)
        self.declare_parameter('max_wz', 1.0)

        self.declare_parameter('pos_tolerance', 0.03)
        self.declare_parameter('yaw_tolerance', 0.05)
        self.declare_parameter('heading_lock_distance', 0.20)

        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        reference_topic = self.get_parameter('reference_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value

        self.control_rate = float(self.get_parameter('control_rate').value)
        self.k_x = float(self.get_parameter('k_x').value)
        self.k_y = float(self.get_parameter('k_y').value)
        self.k_yaw = float(self.get_parameter('k_yaw').value)

        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.max_wz = float(self.get_parameter('max_wz').value)

        self.pos_tolerance = float(self.get_parameter('pos_tolerance').value)
        self.yaw_tolerance = float(self.get_parameter('yaw_tolerance').value)
        self.heading_lock_distance = float(self.get_parameter('heading_lock_distance').value)

        self.current_x: Optional[float] = None
        self.current_y: Optional[float] = None
        self.current_yaw: Optional[float] = None

        self.ref_x: Optional[float] = None
        self.ref_y: Optional[float] = None
        self.ref_yaw: Optional[float] = None

        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        self.ref_sub = self.create_subscription(
            PoseStamped,
            reference_topic,
            self.reference_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            cmd_vel_topic,
            10
        )

        timer_period = 1.0 / self.control_rate
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info('Mecanum pose tracker started')

    def odom_callback(self, msg: Odometry) -> None:
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.current_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def reference_callback(self, msg: PoseStamped) -> None:
        self.ref_x = msg.pose.position.x
        self.ref_y = msg.pose.position.y

        q = msg.pose.orientation
        self.ref_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def publish_zero(self) -> None:
        msg = Twist()
        self.cmd_pub.publish(msg)

    def control_loop(self) -> None:
        if self.current_x is None or self.current_y is None or self.current_yaw is None:
            return

        if self.ref_x is None or self.ref_y is None or self.ref_yaw is None:
            self.publish_zero()
            return

        dx_world = self.ref_x - self.current_x
        dy_world = self.ref_y - self.current_y
        yaw_error = wrap_to_pi(self.ref_yaw - self.current_yaw)

        distance_error = math.hypot(dx_world, dy_world)

        if distance_error < self.pos_tolerance and abs(yaw_error) < self.yaw_tolerance:
            self.publish_zero()
            return

        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)

        dx_body = cos_yaw * dx_world + sin_yaw * dy_world
        dy_body = -sin_yaw * dx_world + cos_yaw * dy_world

        vx_cmd = self.k_x * dx_body
        vy_cmd = self.k_y * dy_body

        if distance_error > self.heading_lock_distance:
            wz_cmd = self.k_yaw * math.atan2(dy_body, max(1e-6, dx_body))
        else:
            wz_cmd = self.k_yaw * yaw_error

        vx_cmd = clamp(vx_cmd, -self.max_vx, self.max_vx)
        vy_cmd = clamp(vy_cmd, -self.max_vy, self.max_vy)
        wz_cmd = clamp(wz_cmd, -self.max_wz, self.max_wz)

        msg = Twist()
        msg.linear.x = vx_cmd
        msg.linear.y = vy_cmd
        msg.angular.z = wz_cmd
        self.cmd_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MecanumPoseTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_zero()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()