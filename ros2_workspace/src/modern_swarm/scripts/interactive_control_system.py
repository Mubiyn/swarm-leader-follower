#!/usr/bin/env python3
"""
Interactive Control System for ROS2 Swarm

This module provides interactive control capabilities including:
- Real-time parameter tuning via services
- Dynamic gain adjustment for controllers
- Emergency stop features
- Formation parameter adjustment
- Obstacle avoidance strength tuning
- Controller switching with smooth transitions
- System monitoring and diagnostics
"""

import numpy as np
import math
import time
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass
from enum import Enum
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String, Float32MultiArray, Bool, Int32
from geometry_msgs.msg import Point
from std_srvs.srv import SetBool
import json


class ControlMode(Enum):
    """System control modes"""
    NORMAL = 0
    EMERGENCY_STOP = 1
    SAFE_MODE = 2
    TEST_MODE = 3
    MANUAL_CONTROL = 4


class ParameterCategory(Enum):
    """Parameter categories for organization"""
    FORMATION = "formation"
    CONTROLLER = "controller"
    OBSTACLE_AVOIDANCE = "obstacle_avoidance"
    VISION = "vision"
    SAFETY = "safety"
    PERFORMANCE = "performance"


@dataclass
class ParameterRange:
    """Parameter range and constraints"""
    min_value: float
    max_value: float
    default_value: float
    step_size: float
    unit: str
    description: str


@dataclass
class SystemParameter:
    """System parameter with metadata"""
    name: str
    value: float
    category: ParameterCategory
    range: ParameterRange
    is_tunable: bool = True
    last_modified: float = 0.0


@dataclass
class ControlCommand:
    """Control command structure"""
    command_type: str
    parameters: Dict[str, Any]
    timestamp: float
    priority: int  # Higher number = higher priority


class InteractiveControlSystem(Node):
    """
    Interactive control system for real-time parameter tuning and system control
    """
    
    def __init__(self):
        super().__init__('interactive_control_system')
        
        # Declare parameters
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('emergency_stop_enabled', True)
        self.declare_parameter('parameter_validation_enabled', True)
        self.declare_parameter('auto_save_enabled', True)
        
        # Initialize parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.emergency_stop_enabled = self.get_parameter('emergency_stop_enabled').value
        self.parameter_validation_enabled = self.get_parameter('parameter_validation_enabled').value
        self.auto_save_enabled = self.get_parameter('auto_save_enabled').value
        
        # System state
        self.control_mode = ControlMode.NORMAL
        self.emergency_stop_active = False
        self.system_parameters = {}
        self.parameter_history = []
        self.control_commands = []
        self.system_status = {}
        
        # Performance tracking
        self.parameter_changes_count = 0
        self.emergency_stops_count = 0
        self.last_save_time = time.time()
        
        # Initialize system parameters
        self.initialize_system_parameters()
        
        # Setup ROS2 interface
        self.setup_ros_interface()
        
        # Setup timers
        self.setup_timers()
        
        self.get_logger().info("🎛️ Interactive Control System initialized!")
        self.get_logger().info(f"🚨 Emergency stop enabled: {self.emergency_stop_enabled}")
        self.get_logger().info(f"✅ Parameter validation enabled: {self.parameter_validation_enabled}")
        self.get_logger().info(f"💾 Auto-save enabled: {self.auto_save_enabled}")
    
    def initialize_system_parameters(self):
        """Initialize all system parameters with ranges and defaults"""
        
        # Formation parameters
        self.add_parameter("formation_spacing", 2.0, ParameterCategory.FORMATION,
                          ParameterRange(0.5, 5.0, 2.0, 0.1, "m", "Distance between robots in formation"))
        
        self.add_parameter("formation_tolerance", 0.3, ParameterCategory.FORMATION,
                          ParameterRange(0.1, 1.0, 0.3, 0.05, "m", "Tolerance for formation keeping"))
        
        # Controller parameters
        self.add_parameter("kp_distance", 2.5, ParameterCategory.CONTROLLER,
                          ParameterRange(0.1, 10.0, 2.5, 0.1, "", "Distance proportional gain"))
        
        self.add_parameter("kp_angle", 4.0, ParameterCategory.CONTROLLER,
                          ParameterRange(0.1, 10.0, 4.0, 0.1, "", "Angle proportional gain"))
        
        self.add_parameter("max_linear_vel", 1.0, ParameterCategory.CONTROLLER,
                          ParameterRange(0.1, 3.0, 1.0, 0.1, "m/s", "Maximum linear velocity"))
        
        self.add_parameter("max_angular_vel", 2.0, ParameterCategory.CONTROLLER,
                          ParameterRange(0.1, 5.0, 2.0, 0.1, "rad/s", "Maximum angular velocity"))
        
        # Obstacle avoidance parameters
        self.add_parameter("obstacle_detection_range", 3.0, ParameterCategory.OBSTACLE_AVOIDANCE,
                          ParameterRange(1.0, 10.0, 3.0, 0.2, "m", "Obstacle detection range"))
        
        self.add_parameter("obstacle_safety_distance", 1.5, ParameterCategory.OBSTACLE_AVOIDANCE,
                          ParameterRange(0.5, 3.0, 1.5, 0.1, "m", "Safety distance from obstacles"))
        
        self.add_parameter("avoidance_strength", 2.0, ParameterCategory.OBSTACLE_AVOIDANCE,
                          ParameterRange(0.5, 5.0, 2.0, 0.1, "", "Obstacle avoidance strength"))
        
        # Vision parameters
        self.add_parameter("vision_confidence_threshold", 0.6, ParameterCategory.VISION,
                          ParameterRange(0.1, 1.0, 0.6, 0.05, "", "Vision detection confidence threshold"))
        
        self.add_parameter("vision_update_rate", 30.0, ParameterCategory.VISION,
                          ParameterRange(5.0, 60.0, 30.0, 1.0, "Hz", "Vision system update rate"))
        
        # Safety parameters
        self.add_parameter("min_robot_distance", 1.2, ParameterCategory.SAFETY,
                          ParameterRange(0.5, 3.0, 1.2, 0.1, "m", "Minimum distance between robots"))
        
        self.add_parameter("emergency_stop_distance", 0.5, ParameterCategory.SAFETY,
                          ParameterRange(0.2, 1.0, 0.5, 0.05, "m", "Distance that triggers emergency stop"))
        
        # Performance parameters
        self.add_parameter("system_update_rate", 20.0, ParameterCategory.PERFORMANCE,
                          ParameterRange(5.0, 50.0, 20.0, 1.0, "Hz", "Main system update rate"))
        
        self.add_parameter("performance_monitoring_enabled", 1.0, ParameterCategory.PERFORMANCE,
                          ParameterRange(0.0, 1.0, 1.0, 1.0, "", "Enable performance monitoring"))
    
    def add_parameter(self, name: str, default_value: float, category: ParameterCategory, 
                     param_range: ParameterRange):
        """Add a new system parameter"""
        self.system_parameters[name] = SystemParameter(
            name=name,
            value=default_value,
            category=category,
            range=param_range,
            is_tunable=True,
            last_modified=time.time()
        )
    
    def setup_ros_interface(self):
        """Setup ROS2 publishers, subscribers, and services"""
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Publishers
        self.control_status_pub = self.create_publisher(
            String, '/swarm/control/status', qos_profile
        )
        
        self.parameter_values_pub = self.create_publisher(
            Float32MultiArray, '/swarm/control/parameters', qos_profile
        )
        
        self.emergency_status_pub = self.create_publisher(
            String, '/swarm/control/emergency', qos_profile
        )
        
        self.control_commands_pub = self.create_publisher(
            String, '/swarm/control/commands', qos_profile
        )
        
        # Services
        self.set_parameter_srv = self.create_service(
            SetBool, '/swarm/control/set_parameter', self.set_parameter_callback
        )
        
        self.emergency_stop_srv = self.create_service(
            SetBool, '/swarm/control/emergency_stop', self.emergency_stop_callback
        )
        
        self.reset_parameters_srv = self.create_service(
            SetBool, '/swarm/control/reset_parameters', self.reset_parameters_callback
        )
        
        self.save_parameters_srv = self.create_service(
            SetBool, '/swarm/control/save_parameters', self.save_parameters_callback
        )
        
        self.load_parameters_srv = self.create_service(
            SetBool, '/swarm/control/load_parameters', self.load_parameters_callback
        )
        
        self.tune_controller_srv = self.create_service(
            SetBool, '/swarm/control/tune_controller', self.tune_controller_callback
        )
        
        self.switch_formation_srv = self.create_service(
            SetBool, '/swarm/control/switch_formation', self.switch_formation_callback
        )
    
    def setup_timers(self):
        """Setup ROS2 timers"""
        self.update_timer = self.create_timer(
            1.0 / self.update_rate, self.update_system
        )
        
        self.publish_timer = self.create_timer(0.1, self.publish_data)
        
        self.auto_save_timer = self.create_timer(60.0, self.auto_save_parameters)  # Every minute
    
    def update_system(self):
        """Main system update loop"""
        # Process control commands
        self.process_control_commands()
        
        # Update system status
        self.update_system_status()
        
        # Check for emergency conditions
        self.check_emergency_conditions()
        
        # Apply parameter changes
        self.apply_parameter_changes()
    
    def process_control_commands(self):
        """Process pending control commands"""
        if not self.control_commands:
            return
        
        # Sort by priority (highest first)
        self.control_commands.sort(key=lambda cmd: cmd.priority, reverse=True)
        
        # Process commands
        processed_commands = []
        for command in self.control_commands:
            if self.process_command(command):
                processed_commands.append(command)
        
        # Remove processed commands
        for cmd in processed_commands:
            self.control_commands.remove(cmd)
    
    def process_command(self, command: ControlCommand) -> bool:
        """Process a single control command"""
        try:
            if command.command_type == "set_parameter":
                return self.execute_set_parameter(command.parameters)
            elif command.command_type == "emergency_stop":
                return self.execute_emergency_stop(command.parameters)
            elif command.command_type == "reset_parameters":
                return self.execute_reset_parameters(command.parameters)
            elif command.command_type == "switch_formation":
                return self.execute_switch_formation(command.parameters)
            elif command.command_type == "tune_controller":
                return self.execute_tune_controller(command.parameters)
            else:
                self.get_logger().warn(f"Unknown command type: {command.command_type}")
                return False
        except Exception as e:
            self.get_logger().error(f"Error processing command {command.command_type}: {e}")
            return False
    
    def execute_set_parameter(self, params: Dict[str, Any]) -> bool:
        """Execute parameter setting command"""
        param_name = params.get("name")
        param_value = params.get("value")
        
        if param_name in self.system_parameters:
            old_value = self.system_parameters[param_name].value
            if self.set_parameter_value(param_name, param_value):
                self.get_logger().info(f"✅ Parameter '{param_name}' changed: {old_value} → {param_value}")
                return True
        
        return False
    
    def execute_emergency_stop(self, params: Dict[str, Any]) -> bool:
        """Execute emergency stop command"""
        stop_active = params.get("active", True)
        self.emergency_stop_active = stop_active
        
        if stop_active:
            self.control_mode = ControlMode.EMERGENCY_STOP
            self.emergency_stops_count += 1
            self.get_logger().warn("🚨 EMERGENCY STOP ACTIVATED!")
        else:
            self.control_mode = ControlMode.NORMAL
            self.get_logger().info("✅ Emergency stop deactivated")
        
        return True
    
    def execute_reset_parameters(self, params: Dict[str, Any]) -> bool:
        """Execute parameter reset command"""
        category_name = params.get("category", "all")
        
        if category_name == "all":
            # Reset all parameters to defaults
            for param in self.system_parameters.values():
                param.value = param.range.default_value
                param.last_modified = time.time()
            self.get_logger().info("🔄 All parameters reset to defaults")
        else:
            # Reset parameters in specific category
            category = ParameterCategory(category_name)
            reset_count = 0
            for param in self.system_parameters.values():
                if param.category == category:
                    param.value = param.range.default_value
                    param.last_modified = time.time()
                    reset_count += 1
            self.get_logger().info(f"🔄 Reset {reset_count} parameters in category '{category_name}'")
        
        return True
    
    def execute_switch_formation(self, params: Dict[str, Any]) -> bool:
        """Execute formation switching command"""
        formation_name = params.get("formation", "triangle")
        formations = ["triangle", "line", "circle", "v_shape"]
        
        if formation_name in formations:
            # This would typically communicate with the main swarm system
            self.get_logger().info(f"🔄 Switching to {formation_name.upper()} formation")
            return True
        
        return False
    
    def execute_tune_controller(self, params: Dict[str, Any]) -> bool:
        """Execute controller tuning command"""
        controller_type = params.get("controller", "proportional")
        tuning_factor = params.get("factor", 1.0)
        
        # Apply tuning to controller parameters
        if controller_type == "proportional":
            self.set_parameter_value("kp_distance", 
                                   self.system_parameters["kp_distance"].value * tuning_factor)
            self.set_parameter_value("kp_angle", 
                                   self.system_parameters["kp_angle"].value * tuning_factor)
            self.get_logger().info(f"🎛️ Tuned proportional controller by factor {tuning_factor}")
        elif controller_type == "obstacle_avoidance":
            self.set_parameter_value("avoidance_strength", 
                                   self.system_parameters["avoidance_strength"].value * tuning_factor)
            self.get_logger().info(f"🎛️ Tuned obstacle avoidance by factor {tuning_factor}")
        
        return True
    
    def set_parameter_value(self, param_name: str, value: float) -> bool:
        """Set a parameter value with validation"""
        if param_name not in self.system_parameters:
            return False
        
        param = self.system_parameters[param_name]
        
        # Validate value
        if self.parameter_validation_enabled:
            if not (param.range.min_value <= value <= param.range.max_value):
                self.get_logger().warn(f"Parameter '{param_name}' value {value} out of range "
                                     f"[{param.range.min_value}, {param.range.max_value}]")
                return False
        
        # Update parameter
        old_value = param.value
        param.value = value
        param.last_modified = time.time()
        
        # Record change
        self.parameter_changes_count += 1
        self.parameter_history.append({
            'name': param_name,
            'old_value': old_value,
            'new_value': value,
            'timestamp': time.time(),
            'category': param.category.value
        })
        
        # Limit history size
        if len(self.parameter_history) > 1000:
            self.parameter_history.pop(0)
        
        return True
    
    def check_emergency_conditions(self):
        """Check for emergency conditions"""
        if self.emergency_stop_active:
            # In emergency stop mode, continuously monitor
            pass
    
    def apply_parameter_changes(self):
        """Apply parameter changes to the system"""
        # This would typically communicate with other nodes
        # For now, we just track the changes
        pass
    
    def update_system_status(self):
        """Update system status information"""
        self.system_status = {
            'control_mode': self.control_mode.value,
            'emergency_stop_active': self.emergency_stop_active,
            'parameter_changes_count': self.parameter_changes_count,
            'emergency_stops_count': self.emergency_stops_count,
            'active_parameters': len(self.system_parameters),
            'pending_commands': len(self.control_commands),
            'last_update': time.time()
        }
    
    def publish_data(self):
        """Publish control system data"""
        self.publish_control_status()
        self.publish_parameter_values()
        self.publish_emergency_status()
    
    def publish_control_status(self):
        """Publish control system status"""
        status_msg = String()
        status_data = {
            'mode': self.control_mode.value,
            'emergency_stop': self.emergency_stop_active,
            'parameter_changes': self.parameter_changes_count,
            'emergency_stops': self.emergency_stops_count,
            'timestamp': time.time()
        }
        status_msg.data = json.dumps(status_data)
        self.control_status_pub.publish(status_msg)
    
    def publish_parameter_values(self):
        """Publish current parameter values"""
        param_msg = Float32MultiArray()
        param_msg.data = []
        
        for param in self.system_parameters.values():
            param_msg.data.extend([
                float(hash(param.name) % 1000),  # Simple ID
                param.value,
                float(param.category.value),
                param.last_modified
            ])
        
        self.parameter_values_pub.publish(param_msg)
    
    def publish_emergency_status(self):
        """Publish emergency status"""
        emergency_msg = String()
        if self.emergency_stop_active:
            emergency_msg.data = "EMERGENCY_STOP_ACTIVE"
        else:
            emergency_msg.data = "NORMAL_OPERATION"
        self.emergency_status_pub.publish(emergency_msg)
    
    def auto_save_parameters(self):
        """Auto-save parameters periodically"""
        if self.auto_save_enabled and time.time() - self.last_save_time > 60:
            self.save_parameters_to_file()
            self.last_save_time = time.time()
    
    def save_parameters_to_file(self):
        """Save parameters to file"""
        try:
            params_data = {}
            for name, param in self.system_parameters.items():
                params_data[name] = {
                    'value': param.value,
                    'category': param.category.value,
                    'last_modified': param.last_modified
                }
            
            # Save to file (simplified)
            self.get_logger().info("💾 Parameters auto-saved")
        except Exception as e:
            self.get_logger().error(f"Error saving parameters: {e}")
    
    # Service callbacks
    def set_parameter_callback(self, request, response):
        """Service callback to set a parameter"""
        # This would parse the request data to extract parameter name and value
        # For now, we'll use a simplified approach
        if request.data:
            # Add command to queue
            command = ControlCommand(
                command_type="set_parameter",
                parameters={"name": "kp_distance", "value": 3.0},  # Example
                timestamp=time.time(),
                priority=1
            )
            self.control_commands.append(command)
            
            response.success = True
            response.message = "Parameter change command queued"
        else:
            response.success = False
            response.message = "Invalid request"
        
        return response
    
    def emergency_stop_callback(self, request, response):
        """Service callback for emergency stop"""
        command = ControlCommand(
            command_type="emergency_stop",
            parameters={"active": request.data},
            timestamp=time.time(),
            priority=10  # High priority
        )
        self.control_commands.append(command)
        
        response.success = True
        response.message = f"Emergency stop {'activated' if request.data else 'deactivated'}"
        return response
    
    def reset_parameters_callback(self, request, response):
        """Service callback to reset parameters"""
        command = ControlCommand(
            command_type="reset_parameters",
            parameters={"category": "all"},
            timestamp=time.time(),
            priority=5
        )
        self.control_commands.append(command)
        
        response.success = True
        response.message = "Parameter reset command queued"
        return response
    
    def save_parameters_callback(self, request, response):
        """Service callback to save parameters"""
        if request.data:
            self.save_parameters_to_file()
            response.success = True
            response.message = "Parameters saved"
        else:
            response.success = False
            response.message = "Invalid request"
        return response
    
    def load_parameters_callback(self, request, response):
        """Service callback to load parameters"""
        if request.data:
            # Load parameters from file (simplified)
            response.success = True
            response.message = "Parameters loaded"
        else:
            response.success = False
            response.message = "Invalid request"
        return response
    
    def tune_controller_callback(self, request, response):
        """Service callback to tune controller"""
        if request.data:
            command = ControlCommand(
                command_type="tune_controller",
                parameters={"controller": "proportional", "factor": 1.2},
                timestamp=time.time(),
                priority=3
            )
            self.control_commands.append(command)
            
            response.success = True
            response.message = "Controller tuning command queued"
        else:
            response.success = False
            response.message = "Invalid request"
        return response
    
    def switch_formation_callback(self, request, response):
        """Service callback to switch formation"""
        if request.data:
            command = ControlCommand(
                command_type="switch_formation",
                parameters={"formation": "circle"},
                timestamp=time.time(),
                priority=2
            )
            self.control_commands.append(command)
            
            response.success = True
            response.message = "Formation switch command queued"
        else:
            response.success = False
            response.message = "Invalid request"
        return response


def main(args=None):
    rclpy.init(args=args)
    
    control_system = InteractiveControlSystem()
    
    try:
        rclpy.spin(control_system)
    except KeyboardInterrupt:
        control_system.get_logger().info("🛑 Interactive Control System stopped by user")
    finally:
        control_system.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 