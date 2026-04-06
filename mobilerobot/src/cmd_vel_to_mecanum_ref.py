#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped


class CmdVelToMecanumRef(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_mecanum_ref')

        self.declare_parameter('input_topic', 'cmd_vel')
        self.declare_parameter('output_topic', 'mecanum_drive_controller/reference')
        self.declare_parameter('frame_id', 'base_link')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        self.sub = self.create_subscription(
            Twist,
            input_topic,
            self.cmd_callback,
            10
        )

        self.pub = self.create_publisher(
            TwistStamped,
            output_topic,
            10
        )

        self.get_logger().info(f'input_topic  = {input_topic}')
        self.get_logger().info(f'output_topic = {output_topic}')
        self.get_logger().info(f'frame_id     = {self.frame_id}')

    def cmd_callback(self, msg: Twist):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.twist = msg
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToMecanumRef()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()