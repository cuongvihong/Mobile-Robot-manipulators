#!/usr/bin/env python3
import copy
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

class MultiTFRelay(Node):
    def __init__(self):
        super().__init__('multi_tf_odometry_to_tf_relay')

        self.pub = self.create_publisher(TFMessage, '/tf', 10)

        self.sub1 = self.create_subscription(
            TFMessage,
            '/robot1/mecanum_drive_controller/tf_odometry',
            lambda msg: self.cb(msg, 'r1'),
            10
        )
        self.sub2 = self.create_subscription(
            TFMessage,
            '/robot2/mecanum_drive_controller/tf_odometry',
            lambda msg: self.cb(msg, 'r2'),
            10
        )
        self.sub3 = self.create_subscription(
            TFMessage,
            '/robot3/mecanum_drive_controller/tf_odometry',
            lambda msg: self.cb(msg, 'r3'),
            10
        )

        self.get_logger().info(
            'Relaying tf_odometry as r1/r2/r3 odom -> mobilebase_link'
        )

    def cb(self, msg, prefix):
        out = TFMessage()

        for t in msg.transforms:
            nt = copy.deepcopy(t)
            nt.header.frame_id = f'{prefix}_odom'
            nt.child_frame_id = f'{prefix}_mobilebase_link'
            out.transforms.append(nt)

        if out.transforms:
            self.pub.publish(out)

def main():
    rclpy.init()
    node = MultiTFRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()