from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    assem_pkg = FindPackageShare('assem')
    assem_moveit_pkg = FindPackageShare('assem_moveit')

    urdf_xacro = PathJoinSubstitution([assem_pkg, 'urdf', 'myrobot.urdf.xacro'])
    rviz_config = PathJoinSubstitution([assem_pkg, 'rviz', 'assem.rviz'])
    controllers_yaml = PathJoinSubstitution([assem_pkg, 'config', 'myrobot_controllers.yaml'])

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', urdf_xacro]),
        value_type=str
    )

    moveit_config = (
        MoveItConfigsBuilder("myrobot", package_name="assem_moveit")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([assem_moveit_pkg, 'launch', 'move_group.launch.py'])
        )
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            controllers_yaml,
        ],
        remappings=[
            ('mecanum_drive_controller/tf_odometry', '/tf'),
            ('/mecanum_drive_controller/tf_odometry', '/tf'),
        ],
    )
    

    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    spawn_mecanum = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecanum_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    spawn_arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    spawn_gripper = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            {'robot_description': robot_description},
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
    )

    test_moveit_node = Node(
        package="myrobot_commander_cpp",
        executable="test_moveit",
        output="screen",
        parameters=[
            {'robot_description': robot_description},
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        spawn_jsb,
        spawn_mecanum,
        spawn_arm,
        spawn_gripper,
        move_group_launch,
        rviz2,
        # test_moveit_node,
    ])