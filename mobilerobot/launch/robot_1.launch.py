from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mobilerobot',
            executable='mecanum_pose_tracker.py',
            namespace='robot_1',
            parameters=[{
                'odom_topic': '/mecanum_drive_controller/odometry',
                'reference_topic': 'tracking_reference',
                'cmd_vel_topic': 'cmd_vel',
                'control_rate': 30.0,
                'k_x': 1.2,
                'k_y': 1.2,
                'k_yaw': 2.0,
                'max_vx': 0.5,
                'max_vy': 0.5,
                'max_wz': 1.0,
                'pos_tolerance': 0.03,
                'yaw_tolerance': 0.05,
                'heading_lock_distance': 0.20,
            }]
        ),

        Node(
            package='mobilerobot',
            executable='cmd_vel_to_mecanum_ref.py',
            namespace='robot_1',
            parameters=[{
                'input_topic': 'cmd_vel',
                'output_topic': '/mecanum_drive_controller/reference',
                'frame_id': 'base_link',
            }]
        ),

        Node(
            package='mobilerobot',
            executable='local_path_follower.py',
            namespace='robot_1',
            parameters=[{
                'odom_topic': '/mecanum_drive_controller/odometry',
                'path_topic': 'local_path',
                'reference_topic': 'tracking_reference',
                'control_rate': 20.0,
                'lookahead_distance': 0.30,
                'goal_tolerance': 0.05,
            }]
        ),

        Node(
            package='mobilerobot',
            executable='route_segmenter.py',
            namespace='robot_1',
            parameters=[{
                'odom_topic': '/mecanum_drive_controller/odometry',
                'global_path_topic': 'global_path',
                'local_path_topic': 'local_path',
                'publish_rate': 10.0,
                'backward_points': 1,
                'segment_length': 1.5,
                'min_points': 3,
                'max_points': 30,
                'verbose': True,
            }]
        ),
        Node(
            package='mobilerobot',
            executable='planner_node.py',
            namespace='robot_1',
            parameters=[{
                'odom_topic': '/mecanum_drive_controller/odometry',
                'goal_topic': 'goal_pose',
                'global_path_topic': 'global_path',
                'publish_rate': 2.0,
                'path_resolution': 0.10,
                'min_points': 2,
                'republish_on_timer': True,
                'verbose': True,
            }]
        ),
        Node(
            package='mobilerobot',
            executable='mission_dispatcher.py',
            name='mission_dispatcher',
            parameters=[{
                'publish_once': True,
                'publish_delay': 2.0,
                'robot_1_goal_x': 2.0,
                'robot_1_goal_y': 0.5,
                'robot_1_goal_yaw': 0.0,
                'robot_2_goal_x': 1.0,
                'robot_2_goal_y': -0.5,
                'robot_2_goal_yaw': 0.0,
            }]
        ),
    ])