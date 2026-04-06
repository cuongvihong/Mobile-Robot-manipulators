from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pid_params_node = Node(
        package='mobilekinematics',
        executable='pid_params.py',   # tên file python (không .py)
        name='pid_params',
        output='screen'
    )

    mecanum_controller_node = Node(
        package='mobilekinematics',
        executable='mecanum_bot.py',  # tên file python
        name='mecanum_controller',
        output='screen'
    )

    return LaunchDescription([
        pid_params_node,
        mecanum_controller_node
    ])