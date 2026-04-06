from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('mobilerobot')
    graph_file = os.path.join(pkg_share, 'config', 'graph.yaml')

    return LaunchDescription([
        Node(
            package='mobilerobot',
            executable='team_task_manager_v5.py',
            name='team_task_manager_v5',
            output='screen',
            parameters=[{
                'robot_names': ['robot_1', 'robot_2', 'robot_3'],
                'team_name': 'team_1',
                'publish_rate': 2.0,
                'verbose': True,
            }]
        ),

        Node(
            package='mobilerobot',
            executable='team_status_manager_v2.py',
            name='team_status_manager_v2',
            output='screen',
            parameters=[{
                'team_name': 'team_1',
                'all_robot_names': ['robot_1', 'robot_2', 'robot_3'],
            }]
        ),

        Node(
            package='mobilerobot',
            executable='team_safety_monitor_v2.py',
            name='team_safety_monitor_v2',
            output='screen',
            parameters=[{
                'team_name': 'team_1',
                'all_robot_names': ['robot_1', 'robot_2', 'robot_3'],
                'publish_rate': 5.0,
                'verbose': True,
            }]
        ),

        Node(
            package='mobilerobot',
            executable='object_attach_manager.py',
            name='object_attach_manager',
            output='screen',
            parameters=[{
                'team_name': 'team_1',
                'all_robot_names': ['robot_1', 'robot_2', 'robot_3'],
            }]
        ),

        Node(
            package='mobilerobot',
            executable='object_pose_to_payload_pose.py',
            name='object_pose_to_payload_pose',
            output='screen',
            parameters=[{
                'team_name': 'team_1',
                'object_pose_topic': '/payload/object_pose',
                'payload_pose_topic': '/team_1/payload_pose',
            }]
        ),

        Node(
            package='mobilerobot',
            executable='payload_planner_v2.py',
            name='payload_planner_v2',
            output='screen',
            parameters=[{
                'team_name': 'team_1',
                'graph_file': graph_file,
            }]
        ),

        Node(
            package='mobilerobot',
            executable='pickup_place_coordinator.py',
            name='pickup_place_coordinator',
            output='screen',
            parameters=[{
                'team_name': 'team_1',
            }]
        ),

        Node(
            package='mobilerobot',
            executable='formation_generator.py',
            name='formation_generator',
            output='screen',
            parameters=[{
                'team_name': 'team_1',
                'carrier_left_name': 'robot_1',
                'carrier_right_name': 'robot_2',
            }]
        ),

        Node(
            package='mobilerobot',
            executable='arm_task_executor.py',
            name='arm_task_executor_robot_1',
            output='screen',
            parameters=[{
                'robot_name': 'robot_1',
                'team_name': 'team_1',
                'waypoint_topic': '/robot_1/waypoint',
                'gripper_cmd_topic': '/robot_1/gripper_cmd',
            }]
        ),
        Node(
            package='mobilerobot',
            executable='arm_task_executor.py',
            name='arm_task_executor_robot_2',
            output='screen',
            parameters=[{
                'robot_name': 'robot_2',
                'team_name': 'team_1',
                'waypoint_topic': '/robot_2/waypoint',
                'gripper_cmd_topic': '/robot_2/gripper_cmd',
            }]
        ),

        Node(
            package='mobilerobot',
            executable='route_segmenter.py',
            name='route_segmenter_robot_1',
            output='screen',
            parameters=[{
                'odom_topic': '/robot_1/odom',
                'global_path_topic': '/robot_1/local_path',
                'local_path_topic': '/robot_1/segmented_local_path',
            }]
        ),
        Node(
            package='mobilerobot',
            executable='local_path_follower.py',
            name='local_path_follower_robot_1',
            output='screen',
            parameters=[{
                'odom_topic': '/robot_1/odom',
                'path_topic': '/robot_1/segmented_local_path',
                'reference_topic': '/robot_1/tracking_reference',
            }]
        ),
        Node(
            package='mobilerobot',
            executable='mecanum_pose_tracker.py',
            name='mecanum_pose_tracker_robot_1',
            output='screen',
            parameters=[{
                'odom_topic': '/robot_1/odom',
                'reference_topic': '/robot_1/tracking_reference',
                'cmd_vel_topic': '/robot_1/cmd_vel',
            }]
        ),
        Node(
            package='mobilerobot',
            executable='cmd_vel_to_mecanum_ref.py',
            name='cmd_vel_to_mecanum_ref_robot_1',
            output='screen',
            parameters=[{
                'input_topic': '/robot_1/cmd_vel',
                'output_topic': '/robot_1/mecanum_drive_controller/reference',
            }]
        ),

        Node(
            package='mobilerobot',
            executable='route_segmenter.py',
            name='route_segmenter_robot_2',
            output='screen',
            parameters=[{
                'odom_topic': '/robot_2/odom',
                'global_path_topic': '/robot_2/local_path',
                'local_path_topic': '/robot_2/segmented_local_path',
            }]
        ),
        Node(
            package='mobilerobot',
            executable='local_path_follower.py',
            name='local_path_follower_robot_2',
            output='screen',
            parameters=[{
                'odom_topic': '/robot_2/odom',
                'path_topic': '/robot_2/segmented_local_path',
                'reference_topic': '/robot_2/tracking_reference',
            }]
        ),
        Node(
            package='mobilerobot',
            executable='mecanum_pose_tracker.py',
            name='mecanum_pose_tracker_robot_2',
            output='screen',
            parameters=[{
                'odom_topic': '/robot_2/odom',
                'reference_topic': '/robot_2/tracking_reference',
                'cmd_vel_topic': '/robot_2/cmd_vel',
            }]
        ),
        Node(
            package='mobilerobot',
            executable='cmd_vel_to_mecanum_ref.py',
            name='cmd_vel_to_mecanum_ref_robot_2',
            output='screen',
            parameters=[{
                'input_topic': '/robot_2/cmd_vel',
                'output_topic': '/robot_2/mecanum_drive_controller/reference',
            }]
        ),
    ])