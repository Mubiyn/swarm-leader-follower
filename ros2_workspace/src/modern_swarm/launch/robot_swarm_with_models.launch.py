#!/usr/bin/env python3
"""
Launch robot_state_publisher for leader and followers with URDF models, plus the unified swarm node and RViz.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('modern_swarm')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'swarm_robot.urdf')
    rviz_config = os.path.join(pkg_dir, 'config', 'working_swarm.rviz')

    # Leader
    leader_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='leader_state_publisher',
        namespace='leader',
        output='screen',
        parameters=[{'robot_description': open(urdf_path).read(), 'frame_prefix': 'leader/'}]
    )
    # Followers
    follower_nodes = []
    for i in range(1, 4):
        follower_nodes.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'follower_{i}_state_publisher',
            namespace=f'follower_{i}',
            output='screen',
            parameters=[{'robot_description': open(urdf_path).read(), 'frame_prefix': f'follower_{i}/'}]
        ))
    # Swarm node
    swarm_node = Node(
        package='modern_swarm',
        executable='unified_swarm_ros2.py',
        name='unified_swarm_system',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    return LaunchDescription([
        leader_state_pub,
        *follower_nodes,
        swarm_node,
        rviz_node
    ]) 