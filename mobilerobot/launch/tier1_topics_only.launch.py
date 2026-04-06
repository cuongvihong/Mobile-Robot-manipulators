from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mobilerobot',
            executable='payload_state_estimator.py',
            name='payload_state_estimator',
            output='screen',
            parameters=[{
                'team_name': 'team_1',
                'carrier_1_name': 'robot_1',
                'carrier_2_name': 'robot_2',
                'publish_rate': 10.0,
                'verbose': True,
            }]
        ),

        Node(
            package='myrobot_commander_cpp',
            executable='arm_waypoint_terminal',
            name='arm_waypoint_terminal_robot_1',
            output='screen',
            parameters=[{
                'waypoint_topic': '/robot_1/waypoint',
            }]
        ),
    ])