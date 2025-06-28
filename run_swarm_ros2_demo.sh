#!/bin/bash
# Unified Swarm ROS2 Demo Script (with 3D robot models)
# Usage: bash run_swarm_ros2_demo.sh

set -e

# 0. Ensure conda environment is active and correct ros2 is used
if [[ -z "$CONDA_PREFIX" ]]; then
  echo "[ERROR] Please activate your conda ROS2 environment before running this script."
  exit 1
fi
ROS2_BIN=$(which ros2)
if [[ "$ROS2_BIN" != "$CONDA_PREFIX"* ]]; then
  echo "[ERROR] ros2 is not from your conda environment."
  echo "  ros2:    $ROS2_BIN"
  echo "[HINT] Run: conda activate ros2"
  exit 1
fi
PYTHON_BIN="/opt/miniconda3/envs/ros2/bin/python3"
if [[ "$(which python3)" != "$PYTHON_BIN" ]]; then
  echo "[WARNING] python3 is not from conda, but will use $PYTHON_BIN explicitly."
fi
PYTHON_VERSION=$($PYTHON_BIN --version 2>&1)
echo "[INFO] Using python: $PYTHON_BIN ($PYTHON_VERSION)"

# 1. Build the ROS2 workspace
cd ros2_workspace
colcon build --symlink-install --packages-select modern_swarm

# 2. Set up environment variables
export AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH:$(pwd)/install
export COLCON_PREFIX_PATH=$COLCON_PREFIX_PATH:$(pwd)/install
export PYTHONPATH=$PYTHONPATH:$(pwd)/install/modern_swarm/lib
export PYTHONPATH=$PYTHONPATH:$(pwd)/../demos/python

URDF_PATH=src/modern_swarm/urdf/swarm_robot.urdf
XACRO_PATH=src/modern_swarm/urdf/swarm_robot.urdf.xacro

# 3. Ensure URDF exists and is not empty, regenerate if needed
if [ ! -s "$URDF_PATH" ]; then
  echo "[INFO] URDF not found or empty. Regenerating from XACRO..."
  if [ ! -f "$XACRO_PATH" ]; then
    echo "[ERROR] XACRO file not found: $XACRO_PATH"
    exit 1
  fi
  ros2 run xacro xacro $XACRO_PATH -o $URDF_PATH
fi
if [ ! -s "$URDF_PATH" ]; then
  echo "[ERROR] URDF file is still missing or empty after regeneration: $URDF_PATH"
  exit 1
fi

# Print first 10 lines of URDF for debug
echo "[INFO] First 10 lines of URDF ($URDF_PATH):"
head -10 "$URDF_PATH"

# 4. Generate correct YAML param files for robot_state_publisher
for role in leader follower_1 follower_2 follower_3; do
  prefix=$role/
  node_name="/${role}/robot_state_publisher"
  cat <<EOF > /tmp/${role}_params.yaml
${node_name}:
  ros__parameters:
    robot_description: |
$(cat $URDF_PATH | sed 's/^/      /')
    frame_prefix: "$prefix"
EOF
done
sync

# 5. Publish static transforms from map to each robot's base_link
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 map leader/base_link &
TF1_PID=$!
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 map follower_1/base_link &
TF2_PID=$!
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 map follower_2/base_link &
TF3_PID=$!
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 map follower_3/base_link &
TF4_PID=$!
sleep 1

# 6. Start robot_state_publisher for leader and followers using ros2 run
ros2 run robot_state_publisher robot_state_publisher --ros-args -r __ns:=/leader --params-file /tmp/leader_params.yaml &
LEADER_PID=$!
ros2 run robot_state_publisher robot_state_publisher --ros-args -r __ns:=/follower_1 --params-file /tmp/follower_1_params.yaml &
F1_PID=$!
ros2 run robot_state_publisher robot_state_publisher --ros-args -r __ns:=/follower_2 --params-file /tmp/follower_2_params.yaml &
F2_PID=$!
ros2 run robot_state_publisher robot_state_publisher --ros-args -r __ns:=/follower_3 --params-file /tmp/follower_3_params.yaml &
F3_PID=$!
sleep 2

# 7. Start the unified swarm ROS2 node (in background)
echo "[INFO] Starting unified_swarm_ros2.py..."
$PYTHON_BIN install/modern_swarm/lib/modern_swarm/unified_swarm_ros2.py &
SWARM_PID=$!
sleep 3

# 8. Start RViz with the working config
echo "[INFO] Launching RViz..."
rviz2 -d src/modern_swarm/config/working_swarm.rviz &
RVIZ_PID=$!

# 9. Wait for user to finish
trap "echo 'Stopping processes...'; kill $LEADER_PID $F1_PID $F2_PID $F3_PID $SWARM_PID $RVIZ_PID $TF1_PID $TF2_PID $TF3_PID $TF4_PID 2>/dev/null" EXIT
wait $SWARM_PID 