#!/bin/bash

# Complete ROS2 Swarm System Demo
# This script demonstrates the full working system

echo "🚀 ROS2 Swarm Leader-Follower System - Complete Demo"
echo "=================================================="

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
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

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if we're in the right directory
if [ ! -f "src/modern_swarm/package.xml" ]; then
    print_error "Please run this script from the ros2_workspace directory"
    exit 1
fi

# Set up ROS2 environment manually
print_status "Setting up ROS2 environment..."
export AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH:$(pwd)/install
export COLCON_PREFIX_PATH=$COLCON_PREFIX_PATH:$(pwd)/install
export PYTHONPATH=$PYTHONPATH:$(pwd)/install/modern_swarm/lib
export PYTHONPATH=$PYTHONPATH:$(pwd)/../demos/python

print_success "Environment configured"

echo ""
print_status "Starting ROS2 Swarm System Demo..."
echo ""

# Start the swarm system in background
print_status "Launching unified swarm system..."
python3 install/modern_swarm/lib/modern_swarm/unified_swarm_ros2.py &
SWARM_PID=$!

# Wait a moment for the system to start
sleep 3

if kill -0 $SWARM_PID 2>/dev/null; then
    print_success "Swarm system started successfully (PID: $SWARM_PID)"
else
    print_error "Failed to start swarm system"
    exit 1
fi

echo ""
print_status "System Status:"
echo "=================="

# Check if nodes are running
if ros2 node list 2>/dev/null | grep -q "unified_swarm_system"; then
    print_success "✓ ROS2 node is running"
else
    print_warning "⚠ ROS2 node not found in node list"
fi

# Check available topics
print_status "Available Topics:"
ros2 topic list 2>/dev/null | grep "/swarm" | while read topic; do
    echo "  📡 $topic"
done

# Check available services
print_status "Available Services:"
ros2 service list 2>/dev/null | grep "/swarm" | while read service; do
    echo "  🔧 $service"
done

echo ""
print_status "Demo Features:"
echo "================"
echo "  🤖 Multi-robot leader-follower system"
echo "  📐 Multiple formation patterns (triangle, line, circle, v-shape)"
echo "  🎛️ Multiple controllers (Proportional, MPC)"
echo "  👁️ Computer vision integration"
echo "  🚧 Obstacle avoidance (static and dynamic)"
echo "  🤝 Robot-to-robot collision avoidance"
echo "  📊 Real-time performance monitoring"
echo "  🎮 Interactive control via ROS2 services"

echo ""
print_status "How to interact with the system:"
echo "====================================="
echo "  • View robot positions: ros2 topic echo /swarm/leader/pose"
echo "  • View formation targets: ros2 topic echo /swarm/formation_targets"
echo "  • View obstacles: ros2 topic echo /swarm/obstacles"
echo "  • View system status: ros2 topic echo /swarm/status"
echo "  • Toggle vision: ros2 service call /swarm/toggle_vision std_srvs/srv/SetBool \"data: true\""
echo "  • Toggle obstacles: ros2 service call /swarm/toggle_obstacles std_srvs/srv/SetBool \"data: true\""
echo "  • Add obstacle: ros2 service call /swarm/add_obstacle std_srvs/srv/SetBool \"data: true\""
echo "  • Change formation: ros2 service call /swarm/set_formation std_srvs/srv/SetBool \"data: true\""
echo "  • Change controller: ros2 service call /swarm/set_controller std_srvs/srv/SetBool \"data: true\""

echo ""
print_status "Visualization Options:"
echo "==========================="
echo "  • RViz: ros2 launch modern_swarm unified_swarm_rviz.launch.py"
echo "  • Gazebo: ros2 launch modern_swarm unified_swarm_gazebo.launch.py"
echo "  • Plot data: Use matplotlib or other plotting tools"

echo ""
print_warning "Press Ctrl+C to stop the demo"
echo ""

# Keep the script running and monitor the system
trap 'echo ""; print_status "Stopping swarm system..."; kill $SWARM_PID 2>/dev/null; print_success "Demo completed!"; exit 0' INT

# Monitor the system
while kill -0 $SWARM_PID 2>/dev/null; do
    sleep 5
    # Show some live data every 30 seconds
    if [ $((SECONDS % 30)) -eq 0 ]; then
        echo ""
        print_status "Live System Data (every 30s):"
        echo "================================"
        # Try to get some topic data
        if ros2 topic echo /swarm/status --once 2>/dev/null | head -1; then
            echo "  ✓ System is publishing data"
        else
            echo "  ⚠ No status data available"
        fi
        echo ""
    fi
done

print_error "Swarm system stopped unexpectedly"
exit 1 