from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    assem_pkg = FindPackageShare("assem")
    assem_moveit_pkg = FindPackageShare("assem_moveitr3")

    urdf_xacro = PathJoinSubstitution([assem_pkg, "urdf", "myrobot.urdf.xacro"])
    srdf_xacro = PathJoinSubstitution([assem_moveit_pkg, "config", "myrobot.srdf.xacro"])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            urdf_xacro,
            " ",
            "prefix:=r3_",
            " ",
            "robot_ns:=robot3",
            " ",
            "controllers_file:=",
            PathJoinSubstitution([assem_pkg, "config", "myrobot_controllers_r3.yaml"]),
        ]),
        value_type=str,
    )

    robot_description_semantic = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            srdf_xacro,
            " ",
            "prefix:=r3_",
        ]),
        value_type=str,
    )

    moveit_config = (
        MoveItConfigsBuilder("myrobot", package_name="assem_moveitr3")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    test_moveitr3_node = Node(
        package="myrobot_commander_cpp",
        executable="test_moveitr3",
        namespace="robot3",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_description},
            {"robot_description_semantic": robot_description_semantic},
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
    )

    delayed_test_moveitr3 = TimerAction(
        period=4.0,
        actions=[test_moveitr3_node]
    )

    return LaunchDescription([
        delayed_test_moveitr3
    ])