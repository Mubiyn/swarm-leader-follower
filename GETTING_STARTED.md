# Getting Started with Modern Swarm Leader-Follower

This guide will help you set up the development environment and get the modernized system running.

## Prerequisites

### System Requirements
- Ubuntu 22.04 LTS (recommended)
- Python 3.9+
- Git

### ROS2 Installation
```bash
# Install ROS2 Humble
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install ros-humble-desktop
```

### Gazebo Installation
```bash
# Install Gazebo Garden (compatible with ROS2 Humble)
sudo apt-get update
sudo apt-get install lsb-release wget gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-garden
```

## Workspace Setup

### 1. Create ROS2 Workspace
```bash
# Navigate to your project directory
cd /path/to/your/modern_swarm_leader_follower

# Create ROS2 workspace structure
mkdir -p ros2_ws/src
cd ros2_ws
```

### 2. Source ROS2
```bash
# Add to your ~/.bashrc for permanent setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Source in current terminal
source /opt/ros/humble/setup.bash
```

### 3. Install Dependencies
```bash
# Install ROS2 packages
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-nav2-bringup \
                 ros-humble-navigation2 ros-humble-robot-localization \
                 ros-humble-vision-opencv ros-humble-cv-bridge

# Install Python packages
pip3 install torch torchvision opencv-python numpy casadi gymnasium stable-baselines3
```

## Project Structure Setup

### 1. Create Package Structure
```bash
cd ros2_ws/src

# Create main package
ros2 pkg create --build-type ament_python modern_swarm_leader_follower \
    --dependencies rclpy geometry_msgs sensor_msgs cv_bridge nav2_msgs

# Create additional packages
ros2 pkg create --build-type ament_python swarm_vision \
    --dependencies rclpy sensor_msgs cv_bridge
ros2 pkg create --build-type ament_python swarm_control \
    --dependencies rclpy geometry_msgs nav2_msgs
ros2 pkg create --build-type ament_python swarm_simulation \
    --dependencies rclpy gazebo_ros
```

### 2. Build Workspace
```bash
cd ros2_ws
colcon build
source install/setup.bash
```

## Quick Start

### 1. Launch Basic Simulation
```bash
# Terminal 1: Launch Gazebo with robots
ros2 launch swarm_simulation multi_robot_gazebo.launch.py

# Terminal 2: Start vision node
ros2 run swarm_vision robot_detector robot1

# Terminal 3: Start controller
ros2 run swarm_control mpc_follower robot2

# Terminal 4: Control leader
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robot1/cmd_vel
```

## Development Workflow

### Phase 1: Foundation (Week 1-2)
1. Set up basic robot models in Gazebo
2. Implement multi-robot spawning
3. Test basic ROS2 communication
4. Create simple teleoperation

### Phase 2: Vision System (Week 3-4)
1. Integrate RGB-D cameras
2. Implement basic robot detection
3. Train CNN for robot recognition
4. Create tracking pipeline

### Phase 3: Control Systems (Week 5-6)
1. Implement MPC controller
2. Set up RL environment
3. Compare with original fuzzy logic
4. Tune parameters

## Troubleshooting

### Common Issues
1. **ROS2 not sourced**: Make sure to source ROS2 in each terminal
2. **Package not found**: Run `colcon build` and source the workspace
3. **Gazebo crashes**: Check GPU drivers and reduce simulation complexity
4. **Python import errors**: Verify all packages are installed with pip3

## Next Steps

Once you have the basic setup working:
1. Check the [Architecture Overview](docs/ARCHITECTURE.md)
2. Read the [Development Guide](docs/DEVELOPMENT.md)
3. Start with Phase 1 tasks in the main README

## Getting Help

- Check the [FAQ](docs/FAQ.md)
- Review original project in `../swarm_follow_leader/`
- Create issues for bugs or questions 