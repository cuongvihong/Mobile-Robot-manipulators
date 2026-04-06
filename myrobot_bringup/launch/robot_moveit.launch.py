from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    myrobot_bringup_pkg = FindPackageShare("myrobot_bringup")

    robot1_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                myrobot_bringup_pkg,
                "launch",
                "robot1_moveit.launch.py"
            ])
        )
    )

    robot2_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                myrobot_bringup_pkg,
                "launch",
                "robot2_moveit.launch.py"
            ])
        )
    )

    robot3_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                myrobot_bringup_pkg,
                "launch",
                "robot3_moveit.launch.py"
            ])
        )
    )

    return LaunchDescription([
        robot1_moveit,
        robot2_moveit,
        robot3_moveit,
    ])