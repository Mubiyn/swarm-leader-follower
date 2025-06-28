#!/bin/bash

echo "🚀 Setting up Modern Swarm ROS2 Workspace..."

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_header() {
    echo -e "${BLUE}$1${NC}"
}

# Step 1: Check if conda is available
print_header "📦 Step 1: Checking Conda Environment..."
if ! command -v conda &> /dev/null; then
    print_error "Conda not found! Please install Miniconda or Anaconda first."
    exit 1
fi

print_status "Conda found: $(conda --version)"

# Step 2: Create or activate ROS2 environment
print_header "🐍 Step 2: Setting up Python Environment..."

ENV_NAME="swarm_ros2"
if conda env list | grep -q "$ENV_NAME"; then
    print_status "Environment '$ENV_NAME' already exists, activating..."
    eval "$(conda shell.bash hook)"
    conda activate $ENV_NAME
else
    print_status "Creating new environment '$ENV_NAME'..."
    conda create -n $ENV_NAME python=3.10 -y
    eval "$(conda shell.bash hook)"
    conda activate $ENV_NAME
fi

# Step 3: Install ROS2 Humble (if not already installed)
print_header "🤖 Step 3: Installing ROS2 Humble..."

# Check if ROS2 is already sourced
if [[ -z "${ROS_DISTRO}" ]]; then
    print_status "Installing ROS2 Humble via conda-forge..."
    
    # Add conda-forge and robostack channels
    conda config --env --add channels conda-forge
    conda config --env --add channels robostack-staging
    conda config --env --set channel_priority strict
    
    # Install ROS2 Humble
    conda install ros-humble-desktop -y
    conda install ros-humble-gazebo-ros-pkgs -y
    conda install ros-humble-joint-state-publisher -y
    conda install ros-humble-robot-state-publisher -y
    conda install ros-humble-xacro -y
    conda install ros-humble-tf2-tools -y
    conda install ros-humble-tf2-ros -y
    conda install ros-humble-image-transport -y
    conda install ros-humble-cv-bridge -y
    
    print_status "✅ ROS2 Humble installed via conda"
else
    print_status "✅ ROS2 $ROS_DISTRO already available"
fi

# Step 4: Install Python dependencies
print_header "📚 Step 4: Installing Python Dependencies..."

pip install --upgrade pip
pip install -r requirements.txt

print_status "✅ Python dependencies installed"

# Step 5: Create workspace structure
print_header "🏗️  Step 5: Setting up ROS2 Workspace..."

# Create workspace if it doesn't exist
WORKSPACE_ROOT="$HOME/ros2_ws"
if [[ ! -d "$WORKSPACE_ROOT" ]]; then
    print_status "Creating ROS2 workspace at $WORKSPACE_ROOT"
    mkdir -p $WORKSPACE_ROOT/src
fi

# Create symlink to our package
PACKAGE_PATH="$WORKSPACE_ROOT/src/modern_swarm"
if [[ ! -L "$PACKAGE_PATH" ]]; then
    print_status "Creating symlink to modern_swarm package..."
    ln -sf "$(pwd)/src/modern_swarm" "$PACKAGE_PATH"
fi

# Step 6: Build the workspace
print_header "🔨 Step 6: Building ROS2 Workspace..."

cd $WORKSPACE_ROOT

# Source ROS2 setup
if [[ -f "/opt/ros/humble/setup.bash" ]]; then
    source /opt/ros/humble/setup.bash
fi

# Build workspace
print_status "Building workspace with colcon..."
colcon build --packages-select modern_swarm

if [[ $? -eq 0 ]]; then
    print_status "✅ Workspace built successfully"
else
    print_error "❌ Workspace build failed"
    exit 1
fi

# Step 7: Create activation script
print_header "⚙️  Step 7: Creating Activation Script..."

cat > "$WORKSPACE_ROOT/activate_modern_swarm.sh" << 'EOF'
#!/bin/bash

# Modern Swarm ROS2 Environment Activation
echo "🤖 Activating Modern Swarm ROS2 Environment..."

# Activate conda environment
eval "$(conda shell.bash hook)"
conda activate swarm_ros2

# Source ROS2 if available from conda
if [[ -n "$CONDA_PREFIX" ]]; then
    if [[ -f "$CONDA_PREFIX/setup.bash" ]]; then
        source "$CONDA_PREFIX/setup.bash"
    elif [[ -f "$CONDA_PREFIX/local_setup.bash" ]]; then
        source "$CONDA_PREFIX/local_setup.bash"
    fi
fi

# Source workspace
if [[ -f "install/setup.bash" ]]; then
    source install/setup.bash
fi

# Set environment variables
export ROS_DOMAIN_ID=42
export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:$(pwd)/src/modern_swarm/models"

echo "✅ Modern Swarm Environment Ready!"
echo ""
echo "Available commands:"
echo "  🚀 Launch Gazebo simulation:"
echo "     ros2 launch modern_swarm gazebo_swarm_simulation.launch.py"
echo ""
echo "  🖥️  Launch RViz visualization:"
echo "     rviz2 -d src/modern_swarm/config/swarm_visualization.rviz"
echo ""
echo "  📊 Monitor topics:"
echo "     ros2 topic list"
echo "     ros2 topic echo /leader/pose"
echo ""
echo "  🎮 Run Python demos:"
echo "     cd src/modern_swarm && python demos/python/simple_demo.py"
echo ""
EOF

chmod +x "$WORKSPACE_ROOT/activate_modern_swarm.sh"

# Step 8: Final setup
print_header "✅ Step 8: Final Configuration..."

cd "$(dirname "$0")"  # Return to original directory

print_status "✅ ROS2 workspace setup complete!"
print_status "📁 Workspace location: $WORKSPACE_ROOT"

echo ""
print_header "🎯 Next Steps:"
echo "1. Run the cleanup script:"
echo "   python cleanup_project.py"
echo ""
echo "2. Navigate to workspace and activate:"
echo "   cd $WORKSPACE_ROOT"
echo "   source activate_modern_swarm.sh"
echo ""
echo "3. Test the installation:"
echo "   ros2 launch modern_swarm gazebo_swarm_simulation.launch.py"
echo ""

print_status "🎉 Setup complete! Ready to run swarm simulations!" 