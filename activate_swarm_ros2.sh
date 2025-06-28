#!/bin/bash
# Modern Swarm + ROS2 Environment Activation

echo "🤖 Activating Modern Swarm ROS2 Environment..."

# Activate conda environment
eval "$(conda shell.bash hook)"
conda activate swarm_ros2

# Clear system PYTHONPATH to avoid conflicts
unset PYTHONPATH
export PYTHONPATH=""

# Check if ROS2 is installed
if ! command -v ros2 &> /dev/null; then
    echo "⚠️  ROS2 not found in conda environment. Installing..."
    conda install ros-humble-desktop ros-humble-gazebo-ros-pkgs -y -q
fi

# Set up ROS2 environment variables
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1

# Source ROS2 setup if available
if [[ -n "$CONDA_PREFIX" ]]; then
    if [[ -f "$CONDA_PREFIX/setup.bash" ]]; then
        source "$CONDA_PREFIX/setup.bash"
    elif [[ -f "$CONDA_PREFIX/local_setup.bash" ]]; then
        source "$CONDA_PREFIX/local_setup.bash"
    fi
fi

# Create and source local ROS2 workspace
WORKSPACE_DIR="$(pwd)/ros2_workspace"
if [[ ! -d "$WORKSPACE_DIR" ]]; then
    echo "📁 Creating local ROS2 workspace..."
    mkdir -p "$WORKSPACE_DIR/src"
    
    # Create symlink to our package
    ln -sf "$(pwd)/src/modern_swarm" "$WORKSPACE_DIR/src/"
    
    cd "$WORKSPACE_DIR"
    colcon build --packages-select modern_swarm
    cd - > /dev/null
fi

# Source the workspace
if [[ -f "$WORKSPACE_DIR/install/setup.bash" ]]; then
    source "$WORKSPACE_DIR/install/setup.bash"
    echo "✅ ROS2 workspace sourced successfully"
else
    echo "⚠️  Building ROS2 workspace..."
    cd "$WORKSPACE_DIR"
    colcon build --packages-select modern_swarm --symlink-install
    if [[ -f "install/setup.bash" ]]; then
        source install/setup.bash
        echo "✅ ROS2 workspace built and sourced"
    fi
    cd - > /dev/null
fi

# Set additional environment variables
export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:$(pwd)/models"
export GAZEBO_RESOURCE_PATH="$GAZEBO_RESOURCE_PATH:$(pwd)/worlds"

echo ""
echo "🎉 Modern Swarm Environment Ready!"
echo ""
echo "📊 Available commands:"
echo "  🚀 Launch basic demo:"
echo "     ros2 launch modern_swarm simple_swarm_demo.launch.py"
echo ""
echo "  🖥️  Launch with RViz:"
echo "     ros2 launch modern_swarm simple_swarm_demo.launch.py use_rviz:=true"
echo ""
echo "  🎮 Python demos:"
echo "     python demos/python/simple_demo.py"
echo "     python demos/python/multi_follower_demo.py"
echo ""
echo "  📋 Check system:"
echo "     ros2 pkg list | grep modern_swarm"
echo "     ros2 topic list"
echo ""
echo "🔧 Workspace: $WORKSPACE_DIR"
