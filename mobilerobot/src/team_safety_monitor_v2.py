#!/usr/bin/env python3
from typing import Dict, List

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String


class TeamSafetyMonitorV2(Node):
    """
    Safety monitor mở rộng:
    - formation_ok
    - payload_progress_ok
    - object_attached
    - active member status
    """

    def __init__(self) -> None:
        super().__init__('team_safety_monitor_v2')

        self.declare_parameter('team_name', 'team_1')
        self.declare_parameter('all_robot_names', ['robot_1', 'robot_2', 'robot_3'])
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('verbose', True)

        self.team_name = str(self.get_parameter('team_name').value)
        self.all_robot_names: List[str] = list(self.get_parameter('all_robot_names').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.verbose = bool(self.get_parameter('verbose').value)

        self.formation_ok = True
        self.payload_progress_ok = True
        self.object_attached = False
        self.task_state = 'IDLE'
        self.member_robots: List[str] = []

        self.robot_status: Dict[str, str] = {r: 'IDLE' for r in self.all_robot_names}

        self.create_subscription(Bool, f'/{self.team_name}/formation_ok', self.formation_cb, 10)
        self.create_subscription(Bool, f'/{self.team_name}/payload_progress_ok', self.progress_cb, 10)
        self.create_subscription(Bool, f'/{self.team_name}/object_attached', self.object_attached_cb, 10)
        self.create_subscription(String, f'/{self.team_name}/task_state', self.state_cb, 10)
        self.create_subscription(String, f'/{self.team_name}/member_robots', self.members_cb, 10)

        self.robot_subs = []
        for robot in self.all_robot_names:
            self.robot_subs.append(
                self.create_subscription(
                    String,
                    f'/{robot}/status',
                    self.make_robot_status_cb(robot),
                    10
                )
            )

        self.hold_pub = self.create_publisher(Bool, f'/{self.team_name}/safety_hold', 10)
        self.abort_pub = self.create_publisher(Bool, f'/{self.team_name}/safety_abort', 10)
        self.status_pub = self.create_publisher(String, f'/{self.team_name}/safety_status', 10)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info('Team safety monitor v2 started')

    def formation_cb(self, msg: Bool) -> None:
        self.formation_ok = msg.data

    def progress_cb(self, msg: Bool) -> None:
        self.payload_progress_ok = msg.data

    def object_attached_cb(self, msg: Bool) -> None:
        self.object_attached = msg.data

    def state_cb(self, msg: String) -> None:
        self.task_state = msg.data

    def members_cb(self, msg: String) -> None:
        text = msg.data.strip()
        if text == '':
            self.member_robots = []
            return
        self.member_robots = [x.strip() for x in text.split(',') if x.strip() != '']

    def make_robot_status_cb(self, robot_name: str):
        def callback(msg: String):
            self.robot_status[robot_name] = msg.data
        return callback

    def any_active_member_invalid(self) -> bool:
        for robot in self.member_robots:
            if self.robot_status.get(robot, 'UNKNOWN') not in ['MOVING', 'WAITING', 'AT_GOAL', 'IDLE']:
                return True
        return False

    def timer_callback(self) -> None:
        hold = False
        abort = False

        if self.task_state in ['TRANSPORTING', 'PRE_PLACE', 'RELEASING']:
            if not self.formation_ok or not self.payload_progress_ok:
                hold = True

        if self.task_state == 'TRANSPORTING':
            if not self.object_attached:
                abort = True

        if self.any_active_member_invalid():
            abort = True

        hold_msg = Bool()
        hold_msg.data = hold
        self.hold_pub.publish(hold_msg)

        abort_msg = Bool()
        abort_msg.data = abort
        self.abort_pub.publish(abort_msg)

        s = String()
        s.data = (
            f'task_state={self.task_state};'
            f'formation_ok={self.formation_ok};'
            f'payload_progress_ok={self.payload_progress_ok};'
            f'object_attached={self.object_attached};'
            f'safety_hold={hold};'
            f'safety_abort={abort}'
        )
        self.status_pub.publish(s)

        if self.verbose:
            self.get_logger().debug(s.data)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeamSafetyMonitorV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()