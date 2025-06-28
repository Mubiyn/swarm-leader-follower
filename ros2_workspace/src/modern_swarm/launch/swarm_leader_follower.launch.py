#!/usr/bin/env python3

"""
ROS2 Launch file for Modern Swarm Leader-Follower System

This launch file starts the complete modernized swarm robotics system with:
- Multiple robot namespaces
- Modern vision nodes (YOLO-based detection)
- MPC-based follower controllers
- Gazebo simulation environment
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    """Generate the launch description for the swarm system"""
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_num_robots = DeclareLaunchArgument(
        'num_robots',
        default_value='3',
        description='Number of follower robots'
    )
    
    declare_world_file = DeclareLaunchArgument(
        'world_file',
        default_value='empty_world.sdf',
        description='Gazebo world file'
    )
    
    declare_use_gazebo = DeclareLaunchArgument(
        'use_gazebo',
        default_value='true',
        description='Launch Gazebo simulation'
    )
    
    declare_robot_model = DeclareLaunchArgument(
        'robot_model',
        default_value='neato',
        description='Robot model (neato, turtlebot3, etc.)'
    )
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    num_robots = LaunchConfiguration('num_robots')
    use_gazebo = LaunchConfiguration('use_gazebo')
    robot_model = LaunchConfiguration('robot_model')
    
    # Gazebo simulation (optional)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_ros'), '/launch/gazebo.launch.py'
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world_file'),
            'verbose': 'true'
        }.items(),
        condition=IfCondition(use_gazebo)
    )
    
    # Leader robot (robot0) - manual control or predefined path
    leader_robot = GroupAction([
        Node(
            package='modern_swarm',
            executable='vision_node.py',
            name='leader_vision',
            namespace='robot0',
            parameters=[{
                'robot_namespace': 'robot0',
                'yolo_model_path': 'yolov8n.pt',
                'detection_confidence': 0.5,
                'use_sim_time': use_sim_time
            }],
            output='screen'
        ),
        
        # Optional: Leader controller for autonomous movement
        Node(
            package='modern_swarm',
            executable='leader_controller_node.py',
            name='leader_controller',
            namespace='robot0',
            parameters=[{
                'robot_namespace': 'robot0',
                'use_sim_time': use_sim_time
            }],
            output='screen',
            condition=IfCondition('false')  # Disabled by default
        )
    ])
    
    # Function to create follower robot nodes
    def create_follower_robot(robot_id):
        robot_ns = f'robot{robot_id}'
        
        return GroupAction([
            # Vision node for robot detection
            Node(
                package='modern_swarm',
                executable='vision_node.py',
                name=f'follower_{robot_id}_vision',
                namespace=robot_ns,
                parameters=[{
                    'robot_namespace': robot_ns,
                    'yolo_model_path': 'yolov8n.pt',
                    'detection_confidence': 0.5,
                    'robot_width': 0.4,
                    'max_detection_range': 10.0,
                    'use_sim_time': use_sim_time
                }],
                output='screen'
            ),
            
            # MPC-based follower controller
            Node(
                package='modern_swarm',
                executable='follower_controller_node.py',
                name=f'follower_{robot_id}_controller',
                namespace=robot_ns,
                parameters=[{
                    'robot_namespace': robot_ns,
                    'desired_distance': 0.75,
                    'prediction_horizon': 10,
                    'control_horizon': 3,
                    'max_linear_vel': 1.0,
                    'max_angular_vel': 2.0,
                    'safety_distance': 0.3,
                    'use_sim_time': use_sim_time
                }],
                output='screen'
            ),
            
            # Formation manager (optional enhancement)
            Node(
                package='modern_swarm',
                executable='formation_manager_node.py',
                name=f'follower_{robot_id}_formation',
                namespace=robot_ns,
                parameters=[{
                    'robot_namespace': robot_ns,
                    'formation_type': 'line',
                    'robot_id': robot_id,
                    'use_sim_time': use_sim_time
                }],
                output='screen',
                condition=IfCondition('false')  # Disabled for now
            )
        ])
    
    # Create follower robots dynamically
    follower_robots = []
    for i in range(1, 4):  # Create robots 1, 2, 3 (robot0 is leader)
        follower_robots.append(create_follower_robot(i))
    
    # RViz visualization (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            FindPackageShare('modern_swarm').find('modern_swarm'),
            'config', 'swarm_visualization.rviz'
        )],
        condition=IfCondition('false'),  # Disabled by default
        output='screen'
    )
    
    # System monitor (optional)
    system_monitor = Node(
        package='modern_swarm',
        executable='system_monitor_node.py',
        name='system_monitor',
        parameters=[{
            'num_robots': num_robots,
            'use_sim_time': use_sim_time
        }],
        output='screen',
        condition=IfCondition('false')  # Disabled for now
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_num_robots)
    ld.add_action(declare_world_file)
    ld.add_action(declare_use_gazebo)
    ld.add_action(declare_robot_model)
    
    # Add Gazebo (if enabled)
    ld.add_action(gazebo_launch)
    
    # Add leader robot
    ld.add_action(leader_robot)
    
    # Add follower robots
    for follower in follower_robots:
        ld.add_action(follower)
    
    # Add optional nodes
    ld.add_action(rviz_node)
    ld.add_action(system_monitor)
    
    return ld


if __name__ == '__main__':
    generate_launch_description() 