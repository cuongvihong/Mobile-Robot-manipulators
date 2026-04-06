from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetUseSimTime
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    assem_pkg = FindPackageShare('assem')
    assem_moveit_pkg = FindPackageShare('assem_moveit')

    urdf_xacro = PathJoinSubstitution([assem_pkg, 'urdf', 'myrobot.urdf.xacro'])
    rviz_config = PathJoinSubstitution([assem_pkg, 'rviz', 'assem.rviz'])

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', urdf_xacro]),
        value_type=str
    )

    moveit_config = (
        MoveItConfigsBuilder("myrobot", package_name="assem_moveit")
        .to_moveit_configs()
    )

    move_group_launch = GroupAction([
        SetUseSimTime(True),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([assem_moveit_pkg, 'launch', 'move_group.launch.py'])
            )
        )
    ])

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_description},
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([
        TimerAction(period=1.0, actions=[move_group_launch]),
        TimerAction(period=2.0, actions=[rviz2]),
    ])