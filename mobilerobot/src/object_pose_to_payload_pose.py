#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped


class ObjectPoseToPayloadPose(Node):
    """
    Bridge object pose thật trong simulator -> /team_x/payload_pose

    Bạn chỉ cần remap / set parameter object_pose_topic đúng với topic object thật trong Gazebo.
    """

    def __init__(self) -> None:
        super().__init__('object_pose_to_payload_pose')

        self.declare_parameter('team_name', 'team_1')
        self.declare_parameter('object_pose_topic', '/payload/object_pose')
        self.declare_parameter('payload_pose_topic', '')
        self.declare_parameter('verbose', True)

        self.team_name = str(self.get_parameter('team_name').value)
        self.object_pose_topic = str(self.get_parameter('object_pose_topic').value)
        self.payload_pose_topic = str(self.get_parameter('payload_pose_topic').value) or f'/{self.team_name}/payload_pose'
        self.verbose = bool(self.get_parameter('verbose').value)

        self.sub = self.create_subscription(
            PoseStamped,
            self.object_pose_topic,
            self.object_pose_callback,
            10
        )

        self.pub = self.create_publisher(
            PoseStamped,
            self.payload_pose_topic,
            10
        )

        self.get_logger().info('ObjectPoseToPayloadPose started')
        self.get_logger().info(f'object_pose_topic = {self.object_pose_topic}')
        self.get_logger().info(f'payload_pose_topic = {self.payload_pose_topic}')

    def object_pose_callback(self, msg: PoseStamped) -> None:
        self.pub.publish(msg)
        if self.verbose:
            self.get_logger().debug(
                f'payload_pose <- object_pose ({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f})'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectPoseToPayloadPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()