#!/usr/bin/env python3
from typing import Dict, List

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool


class ObjectAttachManager(Node):
    """
    Quản lý trạng thái attach/detach của object ở mức team.

    Giai đoạn đầu:
    - chưa gọi plugin Gazebo trực tiếp
    - dùng logic trạng thái mềm để xác nhận object_attached/object_detached

    Sau này có thể thay lõi bằng:
    - service/plugin attach của simulator
    - nhưng giữ nguyên interface output cho team FSM
    """

    def __init__(self) -> None:
        super().__init__('object_attach_manager')

        self.declare_parameter('team_name', 'team_1')
        self.declare_parameter('all_robot_names', ['robot_1', 'robot_2', 'robot_3'])
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('verbose', True)

        self.team_name = str(self.get_parameter('team_name').value)
        self.all_robot_names: List[str] = list(self.get_parameter('all_robot_names').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.verbose = bool(self.get_parameter('verbose').value)

        self.team_state = 'IDLE'
        self.member_robots: List[str] = []

        self.arm_grasp_ok: Dict[str, bool] = {r: False for r in self.all_robot_names}
        self.arm_release_ok: Dict[str, bool] = {r: False for r in self.all_robot_names}

        self.object_attached = False
        self.object_detached = False

        self.team_state_sub = self.create_subscription(
            String,
            f'/{self.team_name}/task_state',
            self.team_state_callback,
            10
        )

        self.member_robots_sub = self.create_subscription(
            String,
            f'/{self.team_name}/member_robots',
            self.member_robots_callback,
            10
        )

        self.flag_subs = []
        for robot in self.all_robot_names:
            self.flag_subs.append(
                self.create_subscription(
                    Bool,
                    f'/{robot}/arm_grasp_ok',
                    self.make_flag_callback(self.arm_grasp_ok, robot),
                    10
                )
            )
            self.flag_subs.append(
                self.create_subscription(
                    Bool,
                    f'/{robot}/arm_release_ok',
                    self.make_flag_callback(self.arm_release_ok, robot),
                    10
                )
            )

        self.attached_pub = self.create_publisher(
            Bool,
            f'/{self.team_name}/object_attached',
            10
        )

        self.detached_pub = self.create_publisher(
            Bool,
            f'/{self.team_name}/object_detached',
            10
        )

        self.debug_pub = self.create_publisher(
            String,
            f'/{self.team_name}/object_attach_status',
            10
        )

        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info('Object attach manager started')

    def team_state_callback(self, msg: String) -> None:
        if msg.data != self.team_state:
            self.team_state = msg.data

            if self.team_state in ['IDLE', 'FORMING', 'GO_TO_PICKUP', 'PRE_GRASP']:
                self.object_attached = False
                self.object_detached = False

            elif self.team_state == 'GRASP_READY':
                self.object_detached = False

            elif self.team_state in ['TRANSPORTING', 'PRE_PLACE']:
                self.object_detached = False

            elif self.team_state == 'RELEASING':
                pass

            elif self.team_state == 'DONE':
                self.object_attached = False
                self.object_detached = True

    def member_robots_callback(self, msg: String) -> None:
        text = msg.data.strip()
        if text == '':
            self.member_robots = []
            return
        self.member_robots = [x.strip() for x in text.split(',') if x.strip() != '']

    def make_flag_callback(self, store: Dict[str, bool], robot_name: str):
        def callback(msg: Bool):
            store[robot_name] = msg.data
        return callback

    def all_active_true(self, store: Dict[str, bool]) -> bool:
        if len(self.member_robots) == 0:
            return False
        for robot in self.member_robots:
            if not store.get(robot, False):
                return False
        return True

    def publish_bool(self, pub, value: bool) -> None:
        msg = Bool()
        msg.data = value
        pub.publish(msg)

    def timer_callback(self) -> None:
        # phase grasp -> attached
        if self.team_state in ['GRASP_READY', 'TRANSPORTING', 'PRE_PLACE'] and self.all_active_true(self.arm_grasp_ok):
            self.object_attached = True
            self.object_detached = False

        # phase release -> detached
        if self.team_state in ['RELEASING', 'DONE'] and self.all_active_true(self.arm_release_ok):
            self.object_attached = False
            self.object_detached = True

        self.publish_bool(self.attached_pub, self.object_attached)
        self.publish_bool(self.detached_pub, self.object_detached)

        dbg = String()
        dbg.data = (
            f'team_state={self.team_state};'
            f'members={",".join(self.member_robots) if self.member_robots else "none"};'
            f'object_attached={self.object_attached};'
            f'object_detached={self.object_detached}'
        )
        self.debug_pub.publish(dbg)

        if self.verbose:
            self.get_logger().debug(dbg.data)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectAttachManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()