#!/usr/bin/env python3
import math
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool, Float64MultiArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class ArmTaskExecutor(Node):
    """
    Arm task executor cho từng robot.
    - nhận phase team
    - tự tính waypoint [x,y,z,phi]
    - publish sang arm stack hiện tại
    - xuất các cờ *_ok cho team FSM

    Giai đoạn đầu:
    - dùng timer settle để xác nhận arm "đã tới"
    - phù hợp để nối nhanh vào pipeline hiện tại
    """

    def __init__(self) -> None:
        super().__init__('arm_task_executor')

        self.declare_parameter('robot_name', 'robot_1')
        self.declare_parameter('team_name', 'team_1')

        self.declare_parameter('odom_topic', '')
        self.declare_parameter('team_role_topic', '')
        self.declare_parameter('waypoint_topic', '')
        self.declare_parameter('gripper_cmd_topic', '')
        self.declare_parameter('debug_topic', '')

        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('verbose', True)

        # offset hình học từ base -> grasp frame
        self.declare_parameter('pickup_forward_offset', 0.35)
        self.declare_parameter('pickup_lateral_offset', 0.20)
        self.declare_parameter('pickup_z', 0.18)
        self.declare_parameter('pickup_phi_deg', -90.0)

        self.declare_parameter('pregrasp_backoff', 0.08)
        self.declare_parameter('place_forward_offset', 0.35)
        self.declare_parameter('place_lateral_offset', 0.20)
        self.declare_parameter('place_z', 0.18)
        self.declare_parameter('place_phi_deg', -90.0)
        self.declare_parameter('release_backoff', 0.10)

        # thời gian giả lập settle
        self.declare_parameter('arm_settle_time_sec', 2.0)
        self.declare_parameter('gripper_settle_time_sec', 1.0)

        self.robot_name = str(self.get_parameter('robot_name').value)
        self.team_name = str(self.get_parameter('team_name').value)

        self.odom_topic = str(self.get_parameter('odom_topic').value) or f'/{self.robot_name}/odom'
        self.team_role_topic = str(self.get_parameter('team_role_topic').value) or f'/{self.robot_name}/team_role'
        self.waypoint_topic = str(self.get_parameter('waypoint_topic').value) or f'/{self.robot_name}/waypoint'
        self.gripper_cmd_topic = str(self.get_parameter('gripper_cmd_topic').value) or f'/{self.robot_name}/gripper_cmd'
        self.debug_topic = str(self.get_parameter('debug_topic').value) or f'/{self.robot_name}/arm_task_debug'

        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.verbose = bool(self.get_parameter('verbose').value)

        self.pickup_forward_offset = float(self.get_parameter('pickup_forward_offset').value)
        self.pickup_lateral_offset = float(self.get_parameter('pickup_lateral_offset').value)
        self.pickup_z = float(self.get_parameter('pickup_z').value)
        self.pickup_phi_deg = float(self.get_parameter('pickup_phi_deg').value)

        self.pregrasp_backoff = float(self.get_parameter('pregrasp_backoff').value)
        self.place_forward_offset = float(self.get_parameter('place_forward_offset').value)
        self.place_lateral_offset = float(self.get_parameter('place_lateral_offset').value)
        self.place_z = float(self.get_parameter('place_z').value)
        self.place_phi_deg = float(self.get_parameter('place_phi_deg').value)
        self.release_backoff = float(self.get_parameter('release_backoff').value)

        self.arm_settle_time_sec = float(self.get_parameter('arm_settle_time_sec').value)
        self.gripper_settle_time_sec = float(self.get_parameter('gripper_settle_time_sec').value)

        self.team_state = 'IDLE'
        self.team_role = 'IDLE_ROLE'

        self.pickup_pose: Optional[PoseStamped] = None
        self.dropoff_pose: Optional[PoseStamped] = None

        self.base_x: Optional[float] = None
        self.base_y: Optional[float] = None
        self.base_yaw: Optional[float] = None

        self.last_command_key = ''
        self.command_start_time: Optional[float] = None
        self.current_action = 'NONE'

        self.arm_pregrasp_ok = False
        self.arm_grasp_ok = False
        self.arm_place_ok = False
        self.arm_release_ok = False

        self.team_state_sub = self.create_subscription(
            String,
            f'/{self.team_name}/task_state',
            self.team_state_callback,
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

        self.role_sub = self.create_subscription(
            String,
            self.team_role_topic,
            self.team_role_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        self.waypoint_pub = self.create_publisher(
            Float64MultiArray,
            self.waypoint_topic,
            10
        )

        self.gripper_pub = self.create_publisher(
            String,
            self.gripper_cmd_topic,
            10
        )

        self.pregrasp_ok_pub = self.create_publisher(
            Bool,
            f'/{self.robot_name}/arm_pregrasp_ok',
            10
        )

        self.grasp_ok_pub = self.create_publisher(
            Bool,
            f'/{self.robot_name}/arm_grasp_ok',
            10
        )

        self.place_ok_pub = self.create_publisher(
            Bool,
            f'/{self.robot_name}/arm_place_ok',
            10
        )

        self.release_ok_pub = self.create_publisher(
            Bool,
            f'/{self.robot_name}/arm_release_ok',
            10
        )

        self.debug_pub = self.create_publisher(
            String,
            self.debug_topic,
            10
        )

        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info(f'ArmTaskExecutor started for {self.robot_name}')
        self.get_logger().info(f'waypoint_topic = {self.waypoint_topic}')
        self.get_logger().info(f'gripper_cmd_topic = {self.gripper_cmd_topic}')

    # ---------------- callbacks ----------------
    def team_state_callback(self, msg: String) -> None:
        if msg.data != self.team_state:
            # reset cờ theo phase mới
            self.team_state = msg.data
            self.reset_phase_flags_for_new_state(msg.data)

    def pickup_callback(self, msg: PoseStamped) -> None:
        self.pickup_pose = msg

    def dropoff_callback(self, msg: PoseStamped) -> None:
        self.dropoff_pose = msg

    def team_role_callback(self, msg: String) -> None:
        self.team_role = msg.data

    def odom_callback(self, msg: Odometry) -> None:
        self.base_x = msg.pose.pose.position.x
        self.base_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.base_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

    # ---------------- helper ----------------
    def reset_phase_flags_for_new_state(self, state: str) -> None:
        if state in ['IDLE', 'FORMING', 'GO_TO_PICKUP']:
            self.arm_pregrasp_ok = False
            self.arm_grasp_ok = False
            self.arm_place_ok = False
            self.arm_release_ok = False
            self.current_action = 'NONE'
            self.last_command_key = ''
            self.command_start_time = None

        elif state == 'PRE_GRASP':
            self.arm_pregrasp_ok = False
            self.current_action = 'NONE'
            self.last_command_key = ''
            self.command_start_time = None

        elif state == 'GRASP_READY':
            self.arm_grasp_ok = False
            self.current_action = 'NONE'
            self.last_command_key = ''
            self.command_start_time = None

        elif state == 'PRE_PLACE':
            self.arm_place_ok = False
            self.current_action = 'NONE'
            self.last_command_key = ''
            self.command_start_time = None

        elif state == 'RELEASING':
            self.arm_release_ok = False
            self.current_action = 'NONE'
            self.last_command_key = ''
            self.command_start_time = None

    def role_sign(self) -> float:
        if self.team_role == 'LEFT_CARRIER':
            return +1.0
        if self.team_role == 'RIGHT_CARRIER':
            return -1.0
        return 0.0

    def publish_bool(self, pub, value: bool) -> None:
        msg = Bool()
        msg.data = value
        pub.publish(msg)

    def publish_all_flags(self) -> None:
        self.publish_bool(self.pregrasp_ok_pub, self.arm_pregrasp_ok)
        self.publish_bool(self.grasp_ok_pub, self.arm_grasp_ok)
        self.publish_bool(self.place_ok_pub, self.arm_place_ok)
        self.publish_bool(self.release_ok_pub, self.arm_release_ok)

    def publish_debug(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.debug_pub.publish(msg)
        if self.verbose:
            self.get_logger().debug(text)

    def publish_waypoint(self, x: float, y: float, z: float, phi_deg: float) -> None:
        msg = Float64MultiArray()
        msg.data = [x, y, z, math.radians(phi_deg)]
        self.waypoint_pub.publish(msg)

    def publish_gripper_cmd(self, cmd: str) -> None:
        msg = String()
        msg.data = cmd
        self.gripper_pub.publish(msg)

    def target_from_object_pose(
        self,
        object_pose: PoseStamped,
        forward_offset: float,
        lateral_offset: float,
        z_value: float,
        phi_deg: float,
        backoff: float = 0.0
    ) -> Tuple[float, float, float, float]:
        """
        Sinh target tương đối đơn giản:
        - dùng object pose trong odom
        - trừ về frame base bằng offset hình học đơn giản
        - output vẫn là x,y,z,phi trong convention arm hiện tại của bạn

        Vì arm stack hiện tại của bạn đã quen kiểu "tọa độ lấy từ mobile rồi trừ offset",
        nên ta giữ tư duy đó.
        """
        if self.base_x is None or self.base_y is None or self.base_yaw is None:
            raise RuntimeError('Base pose unavailable')

        sign = self.role_sign()

        ox = object_pose.pose.position.x
        oy = object_pose.pose.position.y
        oq = object_pose.pose.orientation
        obj_yaw = yaw_from_quaternion(oq.x, oq.y, oq.z, oq.w)

        # offset grasp theo object frame
        dx_obj = forward_offset - backoff
        dy_obj = sign * lateral_offset

        grasp_x_world = ox + dx_obj * math.cos(obj_yaw) - dy_obj * math.sin(obj_yaw)
        grasp_y_world = oy + dx_obj * math.sin(obj_yaw) + dy_obj * math.cos(obj_yaw)

        # world -> base-local 2D
        dx_world = grasp_x_world - self.base_x
        dy_world = grasp_y_world - self.base_y

        cos_yaw = math.cos(self.base_yaw)
        sin_yaw = math.sin(self.base_yaw)

        x_local = cos_yaw * dx_world + sin_yaw * dy_world
        y_local = -sin_yaw * dx_world + cos_yaw * dy_world

        return x_local, y_local, z_value, phi_deg

    def maybe_send_action(self, action_key: str, fn_send) -> None:
        if self.last_command_key == action_key:
            return
        fn_send()
        self.last_command_key = action_key
        self.command_start_time = time.time()
        self.current_action = action_key

    def elapsed_since_command(self) -> float:
        if self.command_start_time is None:
            return 0.0
        return time.time() - self.command_start_time

    # ---------------- main loop ----------------
    def timer_callback(self) -> None:
        self.publish_all_flags()

        if self.team_role not in ['LEFT_CARRIER', 'RIGHT_CARRIER']:
            self.publish_debug(f'{self.robot_name}: inactive role={self.team_role}')
            return

        # PRE_GRASP
        if self.team_state == 'PRE_GRASP':
            if self.pickup_pose is None or self.base_x is None:
                return

            def send_pregrasp():
                x, y, z, phi = self.target_from_object_pose(
                    self.pickup_pose,
                    self.pickup_forward_offset,
                    self.pickup_lateral_offset,
                    self.pickup_z,
                    self.pickup_phi_deg,
                    backoff=self.pregrasp_backoff
                )
                self.publish_waypoint(x, y, z, phi)
                self.publish_debug(
                    f'{self.robot_name}: send PRE_GRASP waypoint [{x:.3f}, {y:.3f}, {z:.3f}, {phi:.1f}]'
                )

            self.maybe_send_action('PRE_GRASP_SEND', send_pregrasp)

            if self.elapsed_since_command() >= self.arm_settle_time_sec:
                self.arm_pregrasp_ok = True

        # GRASP_READY
        elif self.team_state == 'GRASP_READY':
            if self.pickup_pose is None or self.base_x is None:
                return

            if self.current_action not in ['GRASP_SEND', 'GRIPPER_CLOSE']:
                def send_grasp():
                    x, y, z, phi = self.target_from_object_pose(
                        self.pickup_pose,
                        self.pickup_forward_offset,
                        self.pickup_lateral_offset,
                        self.pickup_z,
                        self.pickup_phi_deg,
                        backoff=0.0
                    )
                    self.publish_waypoint(x, y, z, phi)
                    self.publish_debug(
                        f'{self.robot_name}: send GRASP waypoint [{x:.3f}, {y:.3f}, {z:.3f}, {phi:.1f}]'
                    )

                self.maybe_send_action('GRASP_SEND', send_grasp)

            elif self.current_action == 'GRASP_SEND' and self.elapsed_since_command() >= self.arm_settle_time_sec:
                self.publish_gripper_cmd('close')
                self.current_action = 'GRIPPER_CLOSE'
                self.command_start_time = time.time()
                self.publish_debug(f'{self.robot_name}: send gripper close')

            elif self.current_action == 'GRIPPER_CLOSE' and self.elapsed_since_command() >= self.gripper_settle_time_sec:
                self.arm_grasp_ok = True

        # PRE_PLACE
        elif self.team_state == 'PRE_PLACE':
            if self.dropoff_pose is None or self.base_x is None:
                return

            def send_place():
                x, y, z, phi = self.target_from_object_pose(
                    self.dropoff_pose,
                    self.place_forward_offset,
                    self.place_lateral_offset,
                    self.place_z,
                    self.place_phi_deg,
                    backoff=0.0
                )
                self.publish_waypoint(x, y, z, phi)
                self.publish_debug(
                    f'{self.robot_name}: send PRE_PLACE waypoint [{x:.3f}, {y:.3f}, {z:.3f}, {phi:.1f}]'
                )

            self.maybe_send_action('PRE_PLACE_SEND', send_place)

            if self.elapsed_since_command() >= self.arm_settle_time_sec:
                self.arm_place_ok = True

        # RELEASING
        elif self.team_state == 'RELEASING':
            if self.dropoff_pose is None or self.base_x is None:
                return

            if self.current_action not in ['GRIPPER_OPEN', 'RELEASE_RETREAT']:
                self.publish_gripper_cmd('open')
                self.current_action = 'GRIPPER_OPEN'
                self.command_start_time = time.time()
                self.publish_debug(f'{self.robot_name}: send gripper open')

            elif self.current_action == 'GRIPPER_OPEN' and self.elapsed_since_command() >= self.gripper_settle_time_sec:
                x, y, z, phi = self.target_from_object_pose(
                    self.dropoff_pose,
                    self.place_forward_offset,
                    self.place_lateral_offset,
                    self.place_z,
                    self.place_phi_deg,
                    backoff=self.release_backoff
                )
                self.publish_waypoint(x, y, z, phi)
                self.current_action = 'RELEASE_RETREAT'
                self.command_start_time = time.time()
                self.publish_debug(
                    f'{self.robot_name}: send RELEASE retreat waypoint [{x:.3f}, {y:.3f}, {z:.3f}, {phi:.1f}]'
                )

            elif self.current_action == 'RELEASE_RETREAT' and self.elapsed_since_command() >= self.arm_settle_time_sec:
                self.arm_release_ok = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmTaskExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()