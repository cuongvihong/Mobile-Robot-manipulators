from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    assem_pkg = FindPackageShare("assem")
    assem_moveit_pkg = FindPackageShare("assem_moveit")

    urdf_xacro = PathJoinSubstitution([assem_pkg, "urdf", "myrobot.urdf.xacro"])
    srdf_xacro = PathJoinSubstitution([assem_moveit_pkg, "config", "myrobot.srdf.xacro"])

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                FindExecutable(name="xacro"),
                " ",
                urdf_xacro,
                " ",
                "prefix:=r1_",
                " ",
                "robot_ns:=robot1",
                " ",
                "controllers_file:=",
                PathJoinSubstitution([assem_pkg, "config", "myrobot_controllers_r1.yaml"]),
            ]),
            value_type=str,
        )
    }

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command([
                FindExecutable(name="xacro"),
                " ",
                srdf_xacro,
                " ",
                "prefix:=r1_",
            ]),
            value_type=str,
        )
    }

    moveit_config = (
        MoveItConfigsBuilder("myrobot", package_name="assem_moveit")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace="robot1",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            robot_description,
            robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            moveit_config.trajectory_execution,
        ],
    )

    return LaunchDescription([
        TimerAction(period=2.0, actions=[move_group]),
    ])