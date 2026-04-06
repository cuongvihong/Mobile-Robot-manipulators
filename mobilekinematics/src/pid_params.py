#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class PIDParams(Node):
    def __init__(self):
        super().__init__('pid_params')

        self.publisher = self.create_publisher(
            Float32MultiArray, '/pid_params', 10)

        self.declare_parameter("kp", 2.0)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("kd", 0.1)

        self.timer = self.create_timer(0.1, self.publish_params)

    def publish_params(self):
        msg = Float32MultiArray()
        msg.data = [
            self.get_parameter("kp").value,
            self.get_parameter("ki").value,
            self.get_parameter("kd").value
        ]
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = PIDParams()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()