#!/usr/bin/env python3
"""
Simple Gazebo launch for swarm simulation
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    pkg_dir = get_package_share_directory('modern_swarm')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(pkg_dir, 'worlds', 'swarm_arena.world'),
        description='Path to world file'
    )
    
    # Start Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', LaunchConfiguration('world_file')],
        output='screen'
    )
    
    # Unified Swarm System Node
    swarm_node = Node(
        package='modern_swarm',
        executable='unified_swarm_ros2.py',
        name='unified_swarm_system',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'update_rate': 20.0,
            'leader_speed': 1.2,
            'leader_radius': 8.0,
            'num_followers': 3,
            'kp_distance': 2.5,
            'kp_angle': 4.0,
            'max_linear_vel': 1.0,
            'max_angular_vel': 2.0,
            'min_robot_distance': 1.2,
            'obstacle_detection_range': 3.0,
            'obstacle_safety_distance': 1.5,
            'camera_resolution': [640, 480],
            'camera_fov': [-10.0, 10.0, -10.0, 10.0]
        }]
    )
    
    # TF2 Static Transform Publisher (for map frame)
    tf_static = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        world_file_arg,
        gazebo,
        tf_static,
        swarm_node
    ]) 