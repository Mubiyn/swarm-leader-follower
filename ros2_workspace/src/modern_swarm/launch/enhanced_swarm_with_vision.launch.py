#!/usr/bin/env python3
"""
Enhanced Swarm System with Vision Launch File

This launch file starts both the main swarm system and the enhanced vision system
for real-time robot detection and vision-based formation control.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for enhanced swarm with vision"""
    
    # Declare launch arguments
    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='20.0',
        description='System update rate in Hz'
    )
    
    leader_speed_arg = DeclareLaunchArgument(
        'leader_speed',
        default_value='1.2',
        description='Leader robot speed'
    )
    
    vision_detection_arg = DeclareLaunchArgument(
        'vision_detection_enabled',
        default_value='true',
        description='Enable real vision detection'
    )
    
    # Main swarm system node
    swarm_node = Node(
        package='modern_swarm',
        executable='unified_swarm_ros2.py',
        name='unified_swarm_system',
        output='screen',
        parameters=[{
            'update_rate': LaunchConfiguration('update_rate'),
            'leader_speed': LaunchConfiguration('leader_speed'),
            'vision_detection_enabled': LaunchConfiguration('vision_detection_enabled'),
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
    
    # Enhanced vision system node
    vision_node = Node(
        package='modern_swarm',
        executable='enhanced_vision_system.py',
        name='enhanced_vision_system',
        output='screen',
        parameters=[{
            'camera_resolution': [640, 480],
            'camera_fov': [-10.0, 10.0, -10.0, 10.0],
            'detection_confidence_threshold': 0.6,
            'detection_history_length': 10,
            'min_contour_area': 100,
            'max_contour_area': 5000,
            'color_detection_enabled': True,
            'motion_detection_enabled': True,
            'vision_update_rate': 30.0
        }]
    )
    
    return LaunchDescription([
        update_rate_arg,
        leader_speed_arg,
        vision_detection_arg,
        swarm_node,
        vision_node
    ]) 