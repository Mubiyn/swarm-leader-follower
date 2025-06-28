#!/bin/bash
# Unified Swarm ROS2 Demo Script
# This script builds the workspace, sets up the environment, runs the swarm node, and launches RViz.
# Usage: bash run_swarm_ros2_demo.sh

set -e

# 1. Build the ROS2 workspace
cd ros2_workspace
colcon build --symlink-install --packages-select modern_swarm

# 2. Set up environment variables
export AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH:$(pwd)/install
export COLCON_PREFIX_PATH=$COLCON_PREFIX_PATH:$(pwd)/install
export PYTHONPATH=$PYTHONPATH:$(pwd)/install/modern_swarm/lib
export PYTHONPATH=$PYTHONPATH:$(pwd)/../demos/python

# 3. Start the unified swarm ROS2 node (in background)
echo "[INFO] Starting unified_swarm_ros2.py..."
python3 install/modern_swarm/lib/modern_swarm/unified_swarm_ros2.py &
SWARM_PID=$!
sleep 3

# 4. Start RViz with the working config
echo "[INFO] Launching RViz..."
rviz2 -d src/modern_swarm/config/working_swarm.rviz &
RVIZ_PID=$!

# 5. Wait for user to finish
trap "echo 'Stopping processes...'; kill $SWARM_PID $RVIZ_PID 2>/dev/null" EXIT
wait $SWARM_PID 