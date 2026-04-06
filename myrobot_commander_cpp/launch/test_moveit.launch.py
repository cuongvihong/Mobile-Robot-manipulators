from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    myrobot_commander_cpp_pkg = FindPackageShare("myrobot_commander_cpp")

    robot1_test_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                myrobot_commander_cpp_pkg,
                "launch",
                "robot1_test_moveit.launch.py"
            ])
        )
    )

    robot2_test_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                myrobot_commander_cpp_pkg,
                "launch",
                "robot2_test_moveit.launch.py"
            ])
        )
    )

    robot3_test_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                myrobot_commander_cpp_pkg,
                "launch",
                "robot3_test_moveit.launch.py"
            ])
        )
    )

    return LaunchDescription([
        robot1_test_moveit,
        robot2_test_moveit,
        robot3_test_moveit,
    ])