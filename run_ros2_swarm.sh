#!/bin/bash

echo "🚀 Starting ROS2 Modern Swarm System..."

# Activate environment
source activate_swarm_ros2.sh

# Navigate to workspace
cd ros2_workspace/src/modern_swarm

# Set Python path
export PYTHONPATH=$PWD:$PYTHONPATH

echo "📡 Starting swarm controller..."
python3 modern_swarm/swarm_controller.py &
CONTROLLER_PID=$!

sleep 2

echo "�� Starting robot simulators..."
python3 modern_swarm/robot_simulator.py --ros-args -p robot_name:=leader -p initial_x:=0.0 -p initial_y:=0.0 &
python3 modern_swarm/robot_simulator.py --ros-args -p robot_name:=follower_1 -p initial_x:=-2.0 -p initial_y:=-1.5 &
python3 modern_swarm/robot_simulator.py --ros-args -p robot_name:=follower_2 -p initial_x:=-2.0 -p initial_y:=1.5 &
python3 modern_swarm/robot_simulator.py --ros-args -p robot_name:=follower_3 -p initial_x:=-4.0 -p initial_y:=0.0 &

sleep 3

echo ""
echo "✅ ROS2 Swarm System is running!"
echo ""
echo "📊 Available commands:"
echo "  ros2 topic list                                          # List all topics"
echo "  ros2 topic echo /leader/pose                            # Monitor leader"
echo "  ros2 service call /switch_formation std_srvs/srv/SetBool \"{data: true}\"  # Switch formation"
echo "  ros2 topic echo /swarm_status                           # Monitor system"
echo ""
echo "🛑 To stop: killall python3"
echo ""

wait $CONTROLLER_PID
