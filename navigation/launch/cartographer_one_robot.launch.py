from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    lua_name = LaunchConfiguration('lua_name')
    cartographer_config_dir = PathJoinSubstitution(
        [FindPackageShare('myrobot_cartographer_slam'), 'config']
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('robot_name', default_value='robot1'),
        DeclareLaunchArgument('lua_name', default_value='r1_2d.lua'),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            namespace=robot_name,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', lua_name,
            ],
            remappings=[
                ('scan', ["/", robot_name, '/scan']),
                ('odom', ["/", robot_name, '/mecanum_drive_controller/odometry']),
            ]
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            namespace=robot_name,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
        ),
    ])
