# Getting Started on macOS

This guide helps you set up the Modern Swarm Leader-Follower project on macOS.

## Prerequisites

### System Requirements
- macOS 12+ (Monterey or later)
- Python 3.9+
- Homebrew
- Docker Desktop (for simulation)

### Install Homebrew (if not already installed)
```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

## Option 1: Docker-based Development (Recommended)

Since ROS2 native support on macOS can be complex, we'll use Docker for the ROS2 environment while developing the core algorithms locally.

### 1. Install Docker Desktop
```bash
# Download and install Docker Desktop from: https://www.docker.com/products/docker-desktop/
# Or using Homebrew:
brew install --cask docker
```

### 2. Set up Development Environment
```bash
# Install Python dependencies locally for development
brew install python@3.11
pip3 install -r requirements.txt

# Install additional tools
brew install git cmake
```

### 3. Create Docker Environment
```bash
# Create a Dockerfile for ROS2 development
cat > Dockerfile << 'EOF'
FROM osrf/ros:humble-desktop

# Install additional packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install torch torchvision opencv-python numpy casadi gymnasium stable-baselines3

# Set up workspace
WORKDIR /workspace
COPY . /workspace/

# Source ROS2 on container start
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
EOF

# Build the Docker image
docker build -t modern-swarm-ros2 .
```

### 4. Run Development Container
```bash
# Create and run the container with GUI support
docker run -it \
    --name swarm-dev \
    -v $(pwd):/workspace \
    -e DISPLAY=host.docker.internal:0 \
    -p 8080:8080 \
    modern-swarm-ros2 \
    bash
```

## Option 2: Conda-based Local Development

For algorithm development without full ROS2 simulation:

### 1. Install Miniconda
```bash
brew install miniconda
conda init zsh  # or bash if you use bash
```

### 2. Create Development Environment
```bash
# Create conda environment
conda create -n swarm_robotics python=3.11
conda activate swarm_robotics

# Install core dependencies
pip install -r requirements.txt

# Install additional packages for development
conda install jupyter matplotlib seaborn pandas
```

### 3. Set up Development Structure
```bash
# Create a simplified development structure
mkdir -p {src,tests,notebooks,configs,datasets}

# Create development packages
mkdir -p src/{vision,control,simulation,utils}
```

## Development Workflow

### For Full ROS2 Development (Docker)
```bash
# Start the Docker container
docker start swarm-dev
docker exec -it swarm-dev bash

# Inside container:
cd /workspace
source /opt/ros/humble/setup.bash

# Create ROS2 workspace
mkdir -p ros2_ws/src
cd ros2_ws
colcon build
source install/setup.bash
```

### For Algorithm Development (Local)
```bash
# Activate conda environment
conda activate swarm_robotics

# Start Jupyter for experimentation
jupyter lab

# Run Python scripts directly
python src/vision/robot_detector.py
python src/control/mpc_controller.py
```

## Simplified Project Structure

Since we're adapting for macOS, let's create a hybrid structure:

```
modern_swarm_leader_follower/
├── docker/                  # Docker setup for ROS2
│   ├── Dockerfile
│   └── docker-compose.yml
├── src/                     # Core algorithm development
│   ├── vision/              # Computer vision modules
│   ├── control/             # Control algorithms
│   ├── simulation/          # Simulation scripts
│   └── utils/               # Utility functions
├── notebooks/               # Jupyter notebooks for experimentation
├── tests/                   # Unit tests
├── configs/                 # Configuration files
├── datasets/                # Training data
└── requirements.txt         # Python dependencies
```

## Next Steps

1. **Choose your development approach** (Docker for full ROS2 or local for algorithms)
2. **Set up the environment** using the instructions above
3. **Start with Phase 1**: Basic algorithm development
4. **Gradually add ROS2 integration** as needed

## Phase 1: Foundation (macOS-friendly)
- [ ] Set up Python environment
- [ ] Implement basic computer vision algorithms
- [ ] Develop control algorithms in isolation
- [ ] Create simulation using matplotlib/pygame
- [ ] Build unit tests for core functions

## Phase 2: ROS2 Integration
- [ ] Set up Docker environment
- [ ] Port algorithms to ROS2 nodes
- [ ] Test with Gazebo simulation
- [ ] Integrate with robot models

Would you like to proceed with the **Docker approach** for full ROS2 development, or the **local approach** for algorithm development first? 