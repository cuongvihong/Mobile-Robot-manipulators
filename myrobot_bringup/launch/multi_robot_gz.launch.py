from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution,
    Command,
    FindExecutable,
    EnvironmentVariable,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def make_robot(
    robot_name: str,
    prefix: str,
    x: float,
    y: float,
    z: float,
    controllers_yaml: str,
    start_offset: float,
):
    assem_pkg = FindPackageShare("assem")
    urdf_xacro = PathJoinSubstitution([assem_pkg, "urdf", "myrobot.urdf.xacro"])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            urdf_xacro,
            " ",
            "prefix:=", prefix,
            " ",
            "controllers_file:=", controllers_yaml,
            " ",
            "robot_ns:=", robot_name,
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=robot_name,
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True},
        ],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", robot_name,
            "-topic", f"/{robot_name}/robot_description",
            "-x", str(x),
            "-y", str(y),
            "-z", str(z),
        ],
    )

    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "-c", f"/{robot_name}/controller_manager",
            "--controller-manager-timeout", "30",
        ],
    )

    spawn_mecanum = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "mecanum_drive_controller",
            "-c", f"/{robot_name}/controller_manager",
            "--controller-manager-timeout", "30",
        ],
    )

    spawn_arm = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "arm_controller",
            "-c", f"/{robot_name}/controller_manager",
            "--controller-manager-timeout", "30",
        ],
    )

    spawn_gripper = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "gripper_controller",
            "-c", f"/{robot_name}/controller_manager",
            "--controller-manager-timeout", "30",
        ],
    )

    return [
        TimerAction(period=start_offset + 0.0, actions=[robot_state_publisher]),
        TimerAction(period=start_offset + 1.0, actions=[spawn_robot]),
        TimerAction(period=start_offset + 6.0, actions=[spawn_jsb]),
        TimerAction(period=start_offset + 10.0, actions=[spawn_mecanum]),
        TimerAction(period=start_offset + 12.0, actions=[spawn_arm]),
        TimerAction(period=start_offset + 14.0, actions=[spawn_gripper]),
    ]


def generate_launch_description():
    bringup_share = get_package_share_directory("myrobot_bringup")
    assem_share = get_package_share_directory("assem")
    share_root = os.path.dirname(assem_share)

    gz_launch = os.path.join(
        get_package_share_directory("ros_gz_sim"),
        "launch",
        "gz_sim.launch.py",
    )

    world_path = os.path.join(bringup_share, "world", "worldtest.sdf")
    gazebo_config_path = os.path.join(bringup_share, "config", "gazebo_bridge.yaml")

    r1_yaml = os.path.join(assem_share, "config", "myrobot_controllers_r1.yaml")
    r2_yaml = os.path.join(assem_share, "config", "myrobot_controllers_r2.yaml")
    r3_yaml = os.path.join(assem_share, "config", "myrobot_controllers_r3.yaml")

    # Cartographer config lấy từ package navigation
    slam_pkg_share = get_package_share_directory("navigation")
    config_dir = os.path.join(slam_pkg_share, "config")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={
            "gz_args": [world_path, " -r"]
        }.items(),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[
            {"config_file": gazebo_config_path},
            {"use_sim_time": True},
        ],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Neo hiển thị global cho từng map, KHÔNG neo vào odom
    static_tf_r1_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "r1_map"],
    )

    static_tf_r2_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["2", "0", "0", "0", "0", "0", "world", "r2_map"],
    )

    static_tf_r3_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["4", "0", "0", "0", "0", "0", "world", "r3_map"],
    )

    robot1 = make_robot(
        robot_name="robot1",
        prefix="r1_",
        x=0.0,
        y=0.0,
        z=0.02,
        controllers_yaml=r1_yaml,
        start_offset=0.0,
    )

    robot2 = make_robot(
        robot_name="robot2",
        prefix="r2_",
        x=2.0,
        y=0.0,
        z=0.02,
        controllers_yaml=r2_yaml,
        start_offset=18.0,
    )

    robot3 = make_robot(
        robot_name="robot3",
        prefix="r3_",
        x=4.0,
        y=0.0,
        z=0.02,
        controllers_yaml=r3_yaml,
        start_offset=36.0,
    )

    tf_relay = Node(
        package="myrobot_bringup",
        executable="tf_relay.py",
        name="multi_tf_odometry_to_tf_relay",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # SLAM robot 1
    r1 = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_r1',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', 'r1_2d.lua',
        ],
        remappings=[
            ('scan', '/robot1/scan'),
            ('odom', '/robot1/mecanum_drive_controller/odometry'),
            ('submap_list', '/robot1/submap_list'),
        ],
    )

    r1_grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_grid_r1',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
        remappings=[
            ('submap_list', '/robot1/submap_list'),
            ('map', '/robot1/map'),
        ],
    )

    # SLAM robot 2
    r2 = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_r2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', 'r2_2d.lua',
        ],
        remappings=[
            ('scan', '/robot2/scan'),
            ('odom', '/robot2/mecanum_drive_controller/odometry'),
            ('submap_list', '/robot2/submap_list'),
        ],
    )

    r2_grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_grid_r2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
        remappings=[
            ('submap_list', '/robot2/submap_list'),
            ('map', '/robot2/map'),
        ],
    )

    # SLAM robot 3
    r3 = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_r3',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', 'r3_2d.lua',
        ],
        remappings=[
            ('scan', '/robot3/scan'),
            ('odom', '/robot3/mecanum_drive_controller/odometry'),
            ('submap_list', '/robot3/submap_list'),
        ],
    )

    r3_grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_grid_r3',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
        remappings=[
            ('submap_list', '/robot3/submap_list'),
            ('map', '/robot3/map'),
        ],
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=[
                EnvironmentVariable("GZ_SIM_RESOURCE_PATH"),
                TextSubstitution(text=":"),
                TextSubstitution(text=share_root),
            ],
        ),

        gz_sim,
        bridge,
        rviz2,

        static_tf_r1_map,
        static_tf_r2_map,
        static_tf_r3_map,

        *robot1,
        *robot2,
        *robot3,

        # tf_relay lên trước slam
        TimerAction(period=20.0, actions=[tf_relay]),

        # delay slam đủ muộn để TF và lidar frame ổn định hơn
        TimerAction(period=60.0, actions=[r1, r1_grid]),
        TimerAction(period=65.0, actions=[r2, r2_grid]),
        TimerAction(period=70.0, actions=[r3, r3_grid]),
    ])