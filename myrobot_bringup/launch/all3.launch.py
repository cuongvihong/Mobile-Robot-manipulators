from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    myrobot_bringup_pkg = FindPackageShare("myrobot_bringup")
    myrobot_commander_cpp_pkg = FindPackageShare("myrobot_commander_cpp")
    navigation_pkg = FindPackageShare("navigation")

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                myrobot_bringup_pkg,
                "launch",
                "multi_robot_gz.launch.py"
            ])
        )
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                myrobot_bringup_pkg,
                "launch",
                "robot_moveit.launch.py"
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

    # slam_nav2_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([
    #             navigation_pkg,
    #             "launch",
    #             "slam_nav2.launch.py"
    #         ])
    #     )
    # )

    return LaunchDescription([
        gz_launch,
        moveit_launch,
        # slam_nav2_launch,
        test_moveit_launch,
    ])