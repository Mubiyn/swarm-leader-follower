#!/usr/bin/env python3

"""
Simple Swarm Demo Launch File

This launch file creates a basic swarm simulation that works with:
- ROS2 Humble
- Gazebo (optional)
- RViz for visualization

Usage:
  ros2 launch modern_swarm simple_swarm_demo.launch.py
  ros2 launch modern_swarm simple_swarm_demo.launch.py use_gazebo:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Launch arguments
    declare_use_gazebo = DeclareLaunchArgument(
        'use_gazebo',
        default_value='false',
        description='Launch Gazebo simulation'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    # Get launch configurations
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_rviz = LaunchConfiguration('use_rviz')
    
    # Main swarm controller node
    swarm_controller = Node(
        package='modern_swarm',
        executable='swarm_controller',
        name='swarm_controller',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'update_rate': 10.0,
            'formation_type': 'triangle'
        }]
    )
    
    # Robot state simulators (for when not using Gazebo)
    robot_simulators = []
    robot_names = ['leader', 'follower_1', 'follower_2', 'follower_3']
    
    for robot_name in robot_names:
        robot_sim = Node(
            package='modern_swarm',
            executable='robot_simulator',
            name=f'{robot_name}_simulator',
            namespace=robot_name,
            condition=UnlessCondition(use_gazebo),
            parameters=[{
                'robot_name': robot_name,
                'initial_x': 0.0 if robot_name == 'leader' else -2.0,
                'initial_y': 0.0,
                'initial_theta': 0.0
            }],
            output='screen'
        )
        robot_simulators.append(robot_sim)
    
    # Static transform publishers for coordinate frames
    tf_publishers = []
    for robot_name in robot_names:
        tf_pub = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'{robot_name}_tf_publisher',
            arguments=[
                '0', '0', '0', '0', '0', '0',
                'map', f'{robot_name}/base_link'
            ],
            output='screen'
        )
        tf_publishers.append(tf_pub)
    
    # RViz visualization
    rviz_config = os.path.join(
        get_package_share_directory('modern_swarm'),
        'config',
        'swarm_visualization.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
        output='screen'
    )
    
    # Gazebo launch (if enabled)
    gazebo_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'gazebo_ros', 'gazebo.launch.py', 'verbose:=true'],
        condition=IfCondition(use_gazebo),
        output='screen'
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_gazebo)
    ld.add_action(declare_use_rviz)
    
    # Add main controller
    ld.add_action(swarm_controller)
    
    # Add robot simulators (when not using Gazebo)
    for robot_sim in robot_simulators:
        ld.add_action(robot_sim)
    
    # Add transform publishers
    for tf_pub in tf_publishers:
        ld.add_action(tf_pub)
    
    # Add visualization
    ld.add_action(rviz_node)
    
    # Add Gazebo (if enabled)
    ld.add_action(gazebo_launch)
    
    return ld 