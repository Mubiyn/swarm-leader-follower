#!/bin/bash

# Fixed script to run ROS2 Swarm System with proper environment setup

echo "🚀 Starting ROS2 Swarm System (Fixed Environment)..."

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

# Set up ROS2 environment manually (bypassing broken setup script)
print_status "Setting up ROS2 environment..."

# Set ROS2 environment variables
export AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH:$(pwd)/install
export COLCON_PREFIX_PATH=$COLCON_PREFIX_PATH:$(pwd)/install
export PYTHONPATH=$PYTHONPATH:$(pwd)/install/modern_swarm/lib
export PYTHONPATH=$PYTHONPATH:$(pwd)/../demos/python

# Test if the package is now recognized
if ros2 pkg list | grep -q "modern_swarm"; then
    print_success "Package 'modern_swarm' is now recognized by ROS2"
else
    print_warning "Package not found in ros2 pkg list, but continuing..."
fi

# Test if we can run the node directly
print_status "Testing node execution..."
if python3 -c "
import sys
sys.path.append('../demos/python')
try:
    from swarm_core import Robot, ProportionalController
    print('✅ Core imports work')
except ImportError as e:
    print(f'❌ Core imports failed: {e}')
    sys.exit(1)
"; then
    print_success "Core imports working"
else
    print_error "Core imports failed"
    exit 1
fi

echo ""
print_status "Choose an option:"
echo "  1. Run unified swarm system (background)"
echo "  2. Run unified swarm system (foreground)"
echo "  3. Run test integration"
echo "  4. Run simple ROS2 test"
echo "  5. Exit"
echo ""

read -p "Enter your choice (1-5): " choice

case $choice in
    1)
        print_status "Starting unified swarm system in background..."
        python3 install/modern_swarm/lib/modern_swarm/unified_swarm_ros2.py &
        SWARM_PID=$!
        echo "Swarm system started with PID: $SWARM_PID"
        echo "To stop: kill $SWARM_PID"
        echo "To check topics: ros2 topic list"
        echo "To check nodes: ros2 node list"
        ;;
    2)
        print_status "Starting unified swarm system in foreground..."
        python3 install/modern_swarm/lib/modern_swarm/unified_swarm_ros2.py
        ;;
    3)
        print_status "Running integration test..."
        python3 install/modern_swarm/lib/modern_swarm/test_ros2_integration.py
        ;;
    4)
        print_status "Running simple ROS2 test..."
        python3 test_simple_ros2.py
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