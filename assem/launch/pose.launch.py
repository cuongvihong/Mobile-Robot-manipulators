from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    assem_pkg = FindPackageShare("assem")
    myrobot_commander_cpp_pkg = FindPackageShare("myrobot_commander_cpp")

    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                assem_pkg,
                "launch",
                "display.launch.py"
            ])
        )
    )

    test_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                myrobot_commander_cpp_pkg,
                "launch",
                "test_moveit.launch.py"
            ])
        )
    )

    return LaunchDescription([
        display_launch,
        test_moveit_launch,
    ])