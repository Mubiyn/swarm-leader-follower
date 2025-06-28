#!/usr/bin/env python3

"""
Formation Control Launch File for Gazebo Swarm Simulation

This launch file runs the formation control algorithms developed in the Python demos
but adapted to work with real Gazebo robots via ROS2 topics.

Features:
- Leader-follower formation control
- Multiple formation patterns
- Obstacle avoidance
- Computer vision integration
- Performance monitoring

Usage:
  # First launch the Gazebo simulation
  ros2 launch launch/gazebo_swarm_simulation.launch.py
  
  # Then launch the formation control
  ros2 launch launch/formation_control.launch.py
  
  # Or launch both together
  ros2 launch launch/formation_control.launch.py launch_gazebo:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # Launch arguments
    declare_launch_gazebo = DeclareLaunchArgument(
        'launch_gazebo',
        default_value='false',
        description='Launch Gazebo simulation along with formation control'
    )
    
    declare_formation_type = DeclareLaunchArgument(
        'formation_type',
        default_value='triangle',
        description='Initial formation type (triangle, line, circle, v_shape)'
    )
    
    declare_controller_type = DeclareLaunchArgument(
        'controller_type',
        default_value='proportional',
        description='Controller type (proportional, mpc, rl)'
    )
    
    declare_enable_vision = DeclareLaunchArgument(
        'enable_vision',
        default_value='false',
        description='Enable computer vision for leader tracking'
    )
    
    declare_enable_obstacles = DeclareLaunchArgument(
        'enable_obstacles',
        default_value='true',
        description='Enable obstacle avoidance'
    )

    # Get launch configurations
    launch_gazebo = LaunchConfiguration('launch_gazebo')
    formation_type = LaunchConfiguration('formation_type')
    controller_type = LaunchConfiguration('controller_type')
    enable_vision = LaunchConfiguration('enable_vision')
    enable_obstacles = LaunchConfiguration('enable_obstacles')
    
    # Optional Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(os.getcwd(), 'launch', 'gazebo_swarm_simulation.launch.py')
        ]),
        condition=IfCondition(launch_gazebo)
    )
    
    # Formation manager node
    formation_manager_node = Node(
        package='modern_swarm',
        executable='formation_manager_node.py',
        name='formation_manager',
        parameters=[{
            'formation_type': formation_type,
            'num_followers': 3,
            'use_sim_time': True
        }],
        output='screen'
    )
    
    # Leader controller node
    leader_controller_node = Node(
        package='modern_swarm',
        executable='leader_controller_node.py',
        name='leader_controller',
        namespace='leader',
        parameters=[{
            'trajectory_type': 'circular',
            'speed': 0.4,
            'radius': 8.0,
            'enable_vision': enable_vision,
            'use_sim_time': True
        }],
        output='screen'
    )
    
    # Follower controller nodes
    follower_1_controller = Node(
        package='modern_swarm',
        executable='follower_controller_node.py',
        name='follower_controller',
        namespace='follower_1',
        parameters=[{
            'robot_id': 1,
            'leader_topic': '/leader/odom',
            'controller_type': controller_type,
            'formation_type': formation_type,
            'desired_distance': 2.5,
            'enable_obstacles': enable_obstacles,
            'use_sim_time': True
        }],
        output='screen'
    )
    
    follower_2_controller = Node(
        package='modern_swarm',
        executable='follower_controller_node.py',
        name='follower_controller',
        namespace='follower_2',
        parameters=[{
            'robot_id': 2,
            'leader_topic': '/leader/odom',
            'controller_type': controller_type,
            'formation_type': formation_type,
            'desired_distance': 2.5,
            'enable_obstacles': enable_obstacles,
            'use_sim_time': True
        }],
        output='screen'
    )
    
    follower_3_controller = Node(
        package='modern_swarm',
        executable='follower_controller_node.py',
        name='follower_controller',
        namespace='follower_3',
        parameters=[{
            'robot_id': 3,
            'leader_topic': '/leader/odom',
            'controller_type': controller_type,
            'formation_type': formation_type,
            'desired_distance': 2.5,
            'enable_obstacles': enable_obstacles,
            'use_sim_time': True
        }],
        output='screen'
    )
    
    # Vision processing node (if enabled)
    vision_node = Node(
        package='modern_swarm',
        executable='vision_processing_node.py',
        name='vision_processor',
        parameters=[{
            'camera_topic': '/leader/camera/image_raw',
            'detection_confidence': 0.5,
            'enable_vision': enable_vision,
            'use_sim_time': True
        }],
        condition=IfCondition(enable_vision),
        output='screen'
    )
    
    # Obstacle detection node
    obstacle_detector_node = Node(
        package='modern_swarm',
        executable='obstacle_detector_node.py',
        name='obstacle_detector',
        parameters=[{
            'detection_range': 5.0,
            'safety_distance': 1.0,
            'enable_obstacles': enable_obstacles,
            'use_sim_time': True
        }],
        condition=IfCondition(enable_obstacles),
        output='screen'
    )
    
    # Performance monitor node
    performance_monitor_node = Node(
        package='modern_swarm',
        executable='performance_monitor_node.py',
        name='performance_monitor',
        parameters=[{
            'num_robots': 4,
            'monitor_rate': 10.0,
            'use_sim_time': True
        }],
        output='screen'
    )
    
    # System coordinator node (handles formation switching, etc.)
    system_coordinator_node = Node(
        package='modern_swarm',
        executable='system_coordinator_node.py',
        name='system_coordinator',
        parameters=[{
            'formation_type': formation_type,
            'controller_type': controller_type,
            'auto_formation_switch': True,
            'switch_interval': 30.0,  # seconds
            'use_sim_time': True
        }],
        output='screen'
    )
    
    # RQT control panel (optional)
    rqt_control_panel = Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='swarm_control_panel',
        arguments=['--perspective-file', os.path.join(os.getcwd(), 'config', 'swarm_control.perspective')],
        output='screen',
        condition=IfCondition('false')  # Disabled by default
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_launch_gazebo)
    ld.add_action(declare_formation_type)
    ld.add_action(declare_controller_type)
    ld.add_action(declare_enable_vision)
    ld.add_action(declare_enable_obstacles)
    
    # Add optional Gazebo launch
    ld.add_action(gazebo_launch)
    
    # Add core nodes
    ld.add_action(formation_manager_node)
    ld.add_action(leader_controller_node)
    ld.add_action(follower_1_controller)
    ld.add_action(follower_2_controller)
    ld.add_action(follower_3_controller)
    
    # Add optional nodes
    ld.add_action(vision_node)
    ld.add_action(obstacle_detector_node)
    ld.add_action(performance_monitor_node)
    ld.add_action(system_coordinator_node)
    ld.add_action(rqt_control_panel)
    
    return ld

if __name__ == '__main__':
    generate_launch_description() 