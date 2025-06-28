#!/bin/bash

# Modern Swarm Leader-Follower Setup Script
# This script sets up the ROS2 workspace and project structure

set -e  # Exit on any error

echo "🚀 Setting up Modern Swarm Leader-Follower Project"
echo "=================================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if ROS2 is installed
check_ros2() {
    print_status "Checking ROS2 installation..."
    if ! command -v ros2 &> /dev/null; then
        print_error "ROS2 not found. Please install ROS2 Humble first."
        print_status "Follow instructions at: https://docs.ros.org/en/humble/Installation.html"
        exit 1
    fi
    print_status "ROS2 found ✓"
}

# Create ROS2 workspace
setup_workspace() {
    print_status "Setting up ROS2 workspace..."
    
    # Create workspace structure
    mkdir -p ros2_ws/src
    cd ros2_ws
    
    # Source ROS2
    source /opt/ros/humble/setup.bash
    
    print_status "ROS2 workspace created ✓"
}

# Create ROS2 packages
create_packages() {
    print_status "Creating ROS2 packages..."
    
    cd src
    
    # Main package
    ros2 pkg create --build-type ament_python modern_swarm_leader_follower \
        --dependencies rclpy geometry_msgs sensor_msgs cv_bridge nav2_msgs \
        --node-name main_controller
    
    # Vision package
    ros2 pkg create --build-type ament_python swarm_vision \
        --dependencies rclpy sensor_msgs cv_bridge image_transport \
        --node-name robot_detector
    
    # Control package
    ros2 pkg create --build-type ament_python swarm_control \
        --dependencies rclpy geometry_msgs nav2_msgs std_msgs \
        --node-name mpc_follower
    
    # Simulation package
    ros2 pkg create --build-type ament_python swarm_simulation \
        --dependencies rclpy gazebo_ros gazebo_msgs \
        --node-name spawn_robots
    
    cd ..
    print_status "ROS2 packages created ✓"
}

# Install Python dependencies
install_python_deps() {
    print_status "Installing Python dependencies..."
    
    # Check if pip3 is available
    if ! command -v pip3 &> /dev/null; then
        print_error "pip3 not found. Please install Python3 and pip3."
        exit 1
    fi
    
    # Install packages
    pip3 install --user torch torchvision opencv-python numpy casadi gymnasium stable-baselines3 matplotlib seaborn
    
    print_status "Python dependencies installed ✓"
}

# Build workspace
build_workspace() {
    print_status "Building ROS2 workspace..."
    
    # Build packages
    colcon build --symlink-install
    
    if [ $? -eq 0 ]; then
        print_status "Workspace built successfully ✓"
    else
        print_error "Build failed. Check the output above for errors."
        exit 1
    fi
}

# Create environment setup script
create_env_script() {
    print_status "Creating environment setup script..."
    
    cat > setup_env.sh << 'EOF'
#!/bin/bash
# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source $(pwd)/install/setup.bash

echo "🤖 Modern Swarm Leader-Follower environment ready!"
echo "Available commands:"
echo "  ros2 launch swarm_simulation multi_robot_gazebo.launch.py"
echo "  ros2 run swarm_vision robot_detector"
echo "  ros2 run swarm_control mpc_follower"
EOF
    
    chmod +x setup_env.sh
    print_status "Environment script created ✓"
}

# Create initial launch file
create_launch_file() {
    print_status "Creating initial launch file..."
    
    mkdir -p src/swarm_simulation/launch
    
    cat > src/swarm_simulation/launch/multi_robot_gazebo.launch.py << 'EOF'
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    world_file = LaunchConfiguration('world_file')
    
    # Declare launch arguments
    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value='empty.world',
        description='Gazebo world file to load'
    )
    
    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('gazebo_ros'),
            'launch',
            'gazebo.launch.py'
        ]),
        launch_arguments={'world': world_file}.items()
    )
    
    # Spawn robot1 (leader)
    spawn_robot1 = Node(
        package='swarm_simulation',
        executable='spawn_robots',
        name='spawn_robot1',
        arguments=['robot1', '0', '0', '0']
    )
    
    # Spawn robot2 (follower)
    spawn_robot2 = Node(
        package='swarm_simulation',
        executable='spawn_robots',
        name='spawn_robot2',
        arguments=['robot2', '-1', '0', '0']
    )
    
    return LaunchDescription([
        declare_world_file_cmd,
        gazebo,
        spawn_robot1,
        spawn_robot2,
    ])
EOF
    
    print_status "Launch file created ✓"
}

# Main execution
main() {
    print_status "Starting project setup..."
    
    check_ros2
    setup_workspace
    create_packages
    install_python_deps
    build_workspace
    create_env_script
    create_launch_file
    
    echo ""
    echo "🎉 Setup complete!"
    echo "=================================================="
    echo "To get started:"
    echo "1. cd ros2_ws"
    echo "2. source setup_env.sh"
    echo "3. ros2 launch swarm_simulation multi_robot_gazebo.launch.py"
    echo ""
    echo "Happy coding! 🤖"
}

# Run main function
main "$@" 