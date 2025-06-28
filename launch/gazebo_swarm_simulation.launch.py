#!/usr/bin/env python3

"""
Gazebo Multi-Robot Swarm Simulation Launch File

This launch file creates a complete swarm robotics simulation environment with:
- Gazebo world with obstacles and boundaries
- Multiple robots with different colors (leader + followers)
- Robot state publishers and transforms
- Individual robot control interfaces
- RViz visualization (optional)

Usage:
  ros2 launch launch/gazebo_swarm_simulation.launch.py
  ros2 launch launch/gazebo_swarm_simulation.launch.py num_robots:=5 use_rviz:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # Launch arguments
    declare_num_robots = DeclareLaunchArgument(
        'num_robots',
        default_value='4',  # 1 leader + 3 followers
        description='Total number of robots (including leader)'
    )
    
    declare_world_file = DeclareLaunchArgument(
        'world_file',
        default_value='swarm_arena.world',
        description='Gazebo world file name'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    declare_robot_spacing = DeclareLaunchArgument(
        'robot_spacing',
        default_value='2.0',
        description='Initial spacing between robots'
    )

    # Get launch configurations
    num_robots = LaunchConfiguration('num_robots')
    world_file = LaunchConfiguration('world_file')
    use_rviz = LaunchConfiguration('use_rviz')
    robot_spacing = LaunchConfiguration('robot_spacing')
    
    # Build world file path
    world_path = os.path.join(os.getcwd(), 'worlds', 'swarm_arena.world')
    
    # Launch Gazebo with our world
    gazebo_launch = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_path],
        output='screen'
    )
    
    # Robot configurations
    robot_configs = [
        # Leader robot (red)
        {
            'name': 'leader',
            'color': 'red',
            'x': '0.0',
            'y': '0.0',
            'z': '0.1',
            'yaw': '0.0'
        },
        # Follower robots (different colors)
        {
            'name': 'follower_1',
            'color': 'blue',
            'x': '-2.0',
            'y': '-1.5',
            'z': '0.1',
            'yaw': '0.0'
        },
        {
            'name': 'follower_2',
            'color': 'green', 
            'x': '-2.0',
            'y': '1.5',
            'z': '0.1',
            'yaw': '0.0'
        },
        {
            'name': 'follower_3',
            'color': 'orange',
            'x': '-4.0',
            'y': '0.0',
            'z': '0.1',
            'yaw': '0.0'
        }
    ]
    
    # Function to create robot spawning nodes
    def create_robot_nodes(robot_config):
        robot_name = robot_config['name']
        
        # Robot state publisher
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'{robot_name}_state_publisher',
            namespace=robot_name,
            parameters=[{
                'robot_description': get_robot_description(robot_name, robot_config['color']),
                'use_sim_time': True,
                'frame_prefix': f'{robot_name}/'
            }],
            output='screen'
        )
        
        # Spawn robot in Gazebo
        spawn_robot = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=f'spawn_{robot_name}',
            arguments=[
                '-entity', robot_name,
                '-x', robot_config['x'],
                '-y', robot_config['y'], 
                '-z', robot_config['z'],
                '-Y', robot_config['yaw'],
                '-topic', f'{robot_name}/robot_description'
            ],
            output='screen'
        )
        
        # Joint state publisher
        joint_state_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name=f'{robot_name}_joint_state_publisher',
            namespace=robot_name,
            parameters=[{'use_sim_time': True}],
            output='screen'
        )
        
        return [robot_state_publisher, spawn_robot, joint_state_publisher]
    
    # Create nodes for all robots
    robot_nodes = []
    for i, config in enumerate(robot_configs):
        if i < 4:  # Limit to 4 robots for now
            robot_nodes.extend(create_robot_nodes(config))
    
    # RViz visualization (optional)
    rviz_config_path = os.path.join(os.getcwd(), 'config', 'swarm_visualization.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_rviz),
        output='screen'
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_num_robots)
    ld.add_action(declare_world_file)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_robot_spacing)
    
    # Add Gazebo
    ld.add_action(gazebo_launch)
    
    # Add all robot nodes
    for node in robot_nodes:
        ld.add_action(node)
    
    # Add RViz
    ld.add_action(rviz_node)
    
    return ld

def get_robot_description(robot_name, color):
    """Generate robot URDF description with robot-specific parameters."""
    
    # Color mappings for Gazebo materials
    color_materials = {
        'red': 'Gazebo/Red',
        'blue': 'Gazebo/Blue', 
        'green': 'Gazebo/Green',
        'orange': 'Gazebo/Orange',
        'yellow': 'Gazebo/Yellow',
        'purple': 'Gazebo/Purple'
    }
    
    material = color_materials.get(color, 'Gazebo/Blue')
    
    # Load and process URDF
    urdf_path = os.path.join(os.getcwd(), 'urdf', 'swarm_robot.urdf.xacro')
    
    # Process xacro to generate URDF
    try:
        import subprocess
        result = subprocess.run([
            'xacro', urdf_path,
            f'robot_name:={robot_name}',
            f'robot_color:={color}'
        ], capture_output=True, text=True, check=True)
        
        urdf_content = result.stdout
        return urdf_content
        
    except Exception as e:
        print(f"Error processing URDF for {robot_name}: {e}")
        # Fallback to basic URDF if xacro fails
        return get_basic_robot_urdf(robot_name, material)

def get_basic_robot_urdf(robot_name, material):
    """Fallback basic URDF if xacro processing fails."""
    
    return f'''<?xml version="1.0"?>
<robot name="{robot_name}">
  <link name="{robot_name}/base_link">
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.25" length="0.2"/>
      </geometry>
      <material name="robot_color">
        <color rgba="0.2 0.5 1.0 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.25" length="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.15"/>
    </inertial>
  </link>
  
  <link name="{robot_name}/left_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.025"/>
      </geometry>
      <material name="wheel_color">
        <color rgba="0.3 0.3 0.3 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.025"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
  
  <link name="{robot_name}/right_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.025"/>
      </geometry>
      <material name="wheel_color">
        <color rgba="0.3 0.3 0.3 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.025"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
  
  <joint name="{robot_name}/left_wheel_joint" type="continuous">
    <parent link="{robot_name}/base_link"/>
    <child link="{robot_name}/left_wheel"/>
    <origin xyz="0 0.15 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
  
  <joint name="{robot_name}/right_wheel_joint" type="continuous">
    <parent link="{robot_name}/base_link"/>
    <child link="{robot_name}/right_wheel"/>
    <origin xyz="0 -0.15 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
  
  <gazebo>
    <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
      <update_rate>50</update_rate>
      <left_joint>{robot_name}/left_wheel_joint</left_joint>
      <right_joint>{robot_name}/right_wheel_joint</right_joint>
      <wheel_separation>0.3</wheel_separation>
      <wheel_diameter>0.1</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <command_topic>{robot_name}/cmd_vel</command_topic>
      <odometry_topic>{robot_name}/odom</odometry_topic>
      <odometry_frame>{robot_name}/odom</odometry_frame>
      <robot_base_frame>{robot_name}/base_link</robot_base_frame>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
    </plugin>
  </gazebo>
  
  <gazebo reference="{robot_name}/base_link">
    <material>{material}</material>
  </gazebo>
</robot>'''

if __name__ == '__main__':
    generate_launch_description() 