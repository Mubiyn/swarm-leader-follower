#!/bin/bash

# Run ROS2 Swarm System with proper environment setup

echo "🚀 Starting ROS2 Swarm System..."

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

# Check if we're in the right directory
if [ ! -f "src/modern_swarm/package.xml" ]; then
    echo "❌ Please run this script from the ros2_workspace directory"
    exit 1
fi

# Source the workspace
print_status "Sourcing ROS2 workspace..."
source install/setup.bash

# Set Python path for our core modules
print_status "Setting up Python path..."
export PYTHONPATH=$PYTHONPATH:$(pwd)/demos/python
export PYTHONPATH=$PYTHONPATH:$(pwd)/src/modern_swarm/scripts

print_success "Environment setup complete!"

# Check if the swarm system can be imported
print_status "Testing imports..."
python3 -c "
import sys
sys.path.append('demos/python')
sys.path.append('src/modern_swarm/scripts')

try:
    import rclpy
    print('✅ ROS2 imports work')
    
    try:
        from swarm_core import Robot, ProportionalController
        print('✅ Core swarm imports work')
    except ImportError as e:
        print(f'⚠️ Core swarm imports failed: {e}')
        print('This may cause issues with the full system')
        
except ImportError as e:
    print(f'❌ ROS2 import failed: {e}')
    sys.exit(1)
"

if [ $? -ne 0 ]; then
    print_warning "Import test failed, but continuing..."
fi

echo ""
print_status "Choose an option:"
echo "  1. Run with RViz visualization"
echo "  2. Run with Gazebo simulation"
echo "  3. Run simple ROS2 test"
echo "  4. Run integration test"
echo "  5. Exit"
echo ""

read -p "Enter your choice (1-5): " choice

case $choice in
    1)
        print_status "Starting with RViz visualization..."
        ros2 launch modern_swarm unified_swarm_rviz.launch.py
        ;;
    2)
        print_status "Starting with Gazebo simulation..."
        ros2 launch modern_swarm unified_swarm_gazebo.launch.py
        ;;
    3)
        print_status "Running simple ROS2 test..."
        python3 test_simple_ros2.py
        ;;
    4)
        print_status "Running integration test..."
        ros2 run modern_swarm test_ros2_integration
        ;;
    5)
        print_status "Exiting..."
        exit 0
        ;;
    *)
        print_warning "Invalid choice. Exiting..."
        exit 1
        ;;
esac 