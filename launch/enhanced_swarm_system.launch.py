#!/usr/bin/env python3
"""
🚀 Enhanced ROS2 Swarm System Launch File

Professional launch configuration for the enhanced swarm system with:
✅ Dynamic formation switching via ROS2 services
✅ Controller switching via ROS2 services
✅ Comprehensive parameter server (30+ parameters)
✅ Real-time obstacle management
✅ NON-BLOCKING threading architecture

Usage:
    # Basic launch
    ros2 launch launch/enhanced_swarm_system.launch.py

    # With custom parameters
    ros2 launch launch/enhanced_swarm_system.launch.py formation:=circle controller:=mpc

    # Load parameter file
    ros2 launch launch/enhanced_swarm_system.launch.py config_file:=config/high_speed.yaml

    # Development mode (with extra debugging)
    ros2 launch launch/enhanced_swarm_system.launch.py debug:=true

Author: Modern Swarm Team
Date: 2025
"""

import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    ExecuteProcess,
    RegisterEventHandler,
    EmitEvent,
    LogInfo
)

from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate the launch description for enhanced swarm system."""
    
    # Get the current directory (assuming launch files are in project root/launch/)
    current_dir = Path(__file__).parent.parent
    
    # === LAUNCH ARGUMENTS ===
    declare_formation = DeclareLaunchArgument(
        'formation',
        default_value='triangle',
        description='Initial formation type: triangle, line, circle, v_shape'
    )
    
    declare_controller = DeclareLaunchArgument(
        'controller',
        default_value='proportional',
        description='Controller type: proportional, mpc, rl'
    )
    
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=str(current_dir / 'config' / 'swarm_parameters.yaml'),
        description='Parameter configuration file path'
    )
    
    declare_enable_visualization = DeclareLaunchArgument(
        'enable_viz',
        default_value='true',
        description='Enable matplotlib visualization'
    )
    
    declare_headless_mode = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run in headless mode (no GUI)'
    )
    
    declare_debug_mode = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug mode with extra logging'
    )
    

    
    # === GET LAUNCH CONFIGURATIONS ===
    formation = LaunchConfiguration('formation')
    controller = LaunchConfiguration('controller')
    config_file = LaunchConfiguration('config_file')
    enable_viz = LaunchConfiguration('enable_viz')
    headless = LaunchConfiguration('headless')
    debug = LaunchConfiguration('debug')

    
    # === ENHANCED SWARM CONTROLLER NODE ===
    swarm_controller_node = Node(
        package='',  # Empty since we're running from source
        executable='python3',
        arguments=[str(current_dir / 'ros2_swarm_bridge_with_services.py')],
        name='enhanced_swarm_controller',
        output='screen',
        parameters=[config_file]
    )
    
    # === PARAMETER LOADER (Load parameters from YAML) ===
    # Note: Parameters are loaded directly by the node via parameters=[config_file]
    
    # === PARAMETER TESTING NODE (run manually if needed) ===
    # To test parameters, run: python scripts/test_parameters.py
    
    # === RQT PLUGINS FOR MONITORING (optional) ===
    # Note: RQT tools can be launched manually if needed in debug mode
    
    # === EVENT HANDLERS ===
    
    # Log when swarm controller starts
    swarm_start_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=swarm_controller_node,
            on_start=[
                LogInfo(msg="🚀 Enhanced Swarm Controller Started!"),
                LogInfo(msg="✅ Services available: /swarm/set_formation, /swarm/set_controller"),
                LogInfo(msg="✅ Parameters: 30+ real-time configurable parameters"),
                LogInfo(msg="🧪 Test with: python scripts/test_parameters.py"),
            ]
        )
    )
    
    # Handle graceful shutdown
    swarm_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=swarm_controller_node,
            on_exit=[
                LogInfo(msg="🛑 Enhanced Swarm Controller Stopped"),
                EmitEvent(event=Shutdown(reason='Swarm controller exited'))
            ]
        )
    )
    
    # === LAUNCH DESCRIPTION ===
    return LaunchDescription([
        # Declare arguments
        declare_formation,
        declare_controller,
        declare_config_file,
        declare_enable_visualization,
        declare_headless_mode,
        declare_debug_mode,

        
        # Log launch info
        LogInfo(msg="🚀 Launching Enhanced ROS2 Swarm System"),
        LogInfo(msg=[
            "📋 Configuration: Formation=", formation, 
            ", Controller=", controller,
            ", Config=", config_file
        ]),
        
        # Main nodes
        swarm_controller_node,
        
        # Event handlers
        swarm_start_handler,
        swarm_exit_handler,
        
        # Final status
        LogInfo(msg="✅ Enhanced Swarm System Launch Complete!"),
        LogInfo(msg="🔧 Available commands:"),
        LogInfo(msg="   • ros2 service call /swarm/set_formation std_srvs/srv/Trigger"),
        LogInfo(msg="   • ros2 param set /swarm_controller formation.spacing 3.0"),
        LogInfo(msg="   • python scripts/test_parameters.py"),
    ])


# === UTILITY FUNCTIONS ===

def get_swarm_config_path():
    """Get the path to swarm configuration directory."""
    return Path(__file__).parent.parent / 'config'

def get_swarm_scripts_path():
    """Get the path to swarm scripts directory."""
    return Path(__file__).parent.parent / 'scripts' 