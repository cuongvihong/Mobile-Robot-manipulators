#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Float64MultiArray

class MecanumController(Node):

    def __init__(self):
        super().__init__('mecanum_controller')

        # Publisher → gửi velocity trực tiếp
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/forward_velocity_controller/commands',
            10
        )

        # Subscribers
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.create_subscription(Float32MultiArray, '/pid_params', self.pid_callback, 10)

        self.timer = self.create_timer(0.01, self.control_loop)

        # Robot params
        self.r = 0.049
        self.L = 0.12
        self.W = 0.12

        # State
        self.vx = 0.0
        self.vy = 0.0
        self.wz_des = 0.0

        self.wheel_current = [0.0]*4

        # PID
        self.kp = 2.0
        self.ki = 0.0
        self.kd = 0.1

        self.prev_error = 0.0
        self.e_int = 0.0

    def cmd_callback(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.wz_des = msg.angular.z

    def joint_callback(self, msg):
        self.wheel_current = list(msg.velocity[:4])

    def pid_callback(self, msg):
        self.kp, self.ki, self.kd = msg.data

    def pid(self, target, current, dt):
        error = target - current
        self.e_int += error * dt
        d = (error - self.prev_error)/dt
        self.prev_error = error
        return self.kp*error + self.ki*self.e_int + self.kd*d

    def ik(self, vx, vy, wz):
        LpW = self.L + self.W
        r = self.r

        v_fl = (vx - vy - LpW*wz)/r
        v_fr = (vx + vy + LpW*wz)/r
        v_rl = (vx + vy - LpW*wz)/r
        v_rr = (vx - vy + LpW*wz)/r

        return [v_fl, v_fr, v_rl, v_rr]

    def control_loop(self):
        dt = 0.01

        # IK
        wheel_target = self.ik(self.vx, self.vy, self.wz_des)

        # PID từng bánh
        cmd = []
        for i in range(4):
            u = self.pid(wheel_target[i], self.wheel_current[i], dt)
            u = np.clip(u, -10.0, 10.0)
            cmd.append(u)

        msg = Float64MultiArray()
        msg.data = cmd
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = MecanumController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()