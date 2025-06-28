#!/usr/bin/env python3
"""
Launch file for Unified Swarm System with RViz visualization
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('modern_swarm')
    
    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_dir, 'config', 'swarm_visualization.rviz'),
        description='Path to RViz config file'
    )
    
    # Unified Swarm System Node
    swarm_node = Node(
        package='modern_swarm',
        executable='unified_swarm_ros2.py',
        name='unified_swarm_system',
        output='screen',
        parameters=[{
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
    
    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=LaunchConfiguration('use_rviz'),
        output='screen'
    )
    
    # TF2 Static Transform Publisher (for map frame)
    tf_static = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )
    
    # Camera Frame TF Publisher
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        arguments=['0', '0', '5', '0', '0', '0', 'map', 'camera_frame'],
        output='screen'
    )
    
    return LaunchDescription([
        use_rviz_arg,
        rviz_config_arg,
        tf_static,
        camera_tf,
        swarm_node,
        rviz_node
    ]) 