from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('myrobot_cartographer_slam')
    config_dir = os.path.join(pkg_share, 'config')

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
        r1, r1_grid,
        r2, r2_grid,
        r3, r3_grid,
    ])