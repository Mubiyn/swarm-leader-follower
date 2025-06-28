#!/bin/bash

# Modern Swarm Leader-Follower Setup Script for macOS
# This script sets up the development environment on macOS

set -e  # Exit on any error

echo "🍎 Setting up Modern Swarm Leader-Follower Project on macOS"
echo "=========================================================="

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
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_choice() {
    echo -e "${BLUE}[CHOICE]${NC} $1"
}

# Check if Homebrew is installed
check_homebrew() {
    print_status "Checking Homebrew installation..."
    if ! command -v brew &> /dev/null; then
        print_error "Homebrew not found. Installing Homebrew..."
        /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
    fi
    print_status "Homebrew found ✓"
}

# Install Python and basic tools
install_basics() {
    print_status "Installing Python and basic tools..."
    
    # Install Python 3.11
    brew install python@3.11 git cmake
    
    # Create symbolic link for python3 if needed
    if ! command -v python3 &> /dev/null; then
        ln -sf /opt/homebrew/bin/python3.11 /usr/local/bin/python3
        ln -sf /opt/homebrew/bin/pip3.11 /usr/local/bin/pip3
    fi
    
    print_status "Basic tools installed ✓"
}

# Set up Python environment
setup_python_env() {
    print_status "Setting up Python environment..."
    
    # Check if conda is available
    if command -v conda &> /dev/null; then
        print_choice "Found conda. Would you like to use conda environment? (y/n)"
        read -r use_conda
        if [[ $use_conda =~ ^[Yy]$ ]]; then
            setup_conda_env
            return
        fi
    fi
    
    # Use pip/venv
    setup_venv_env
}

# Set up conda environment
setup_conda_env() {
    print_status "Creating conda environment 'swarm_robotics'..."
    
    conda create -n swarm_robotics python=3.11 -y
    
    print_status "Activating conda environment..."
    source "$(conda info --base)/etc/profile.d/conda.sh"
    conda activate swarm_robotics
    
    # Install packages
    pip install -r requirements.txt
    conda install jupyter matplotlib seaborn pandas -y
    
    # Create activation script
    cat > activate_env.sh << 'EOF'
#!/bin/bash
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate swarm_robotics
echo "🤖 Conda environment 'swarm_robotics' activated!"
echo "Available commands:"
echo "  jupyter lab                    # Start Jupyter Lab"
echo "  python src/vision/demo.py      # Run vision demo"
echo "  python src/control/demo.py     # Run control demo"
EOF
    
    chmod +x activate_env.sh
    print_status "Conda environment created ✓"
}

# Set up virtual environment
setup_venv_env() {
    print_status "Creating Python virtual environment..."
    
    python3 -m venv venv
    source venv/bin/activate
    
    # Upgrade pip
    pip install --upgrade pip
    
    # Install packages
    pip install -r requirements.txt
    pip install jupyter
    
    # Create activation script
    cat > activate_env.sh << 'EOF'
#!/bin/bash
source venv/bin/activate
echo "🤖 Virtual environment activated!"
echo "Available commands:"
echo "  jupyter lab                    # Start Jupyter Lab"
echo "  python src/vision/demo.py      # Run vision demo"
echo "  python src/control/demo.py     # Run control demo"
EOF
    
    chmod +x activate_env.sh
    print_status "Virtual environment created ✓"
}

# Create initial demo files
create_demo_files() {
    print_status "Creating demo files..."
    
    # Vision demo
    cat > src/vision/demo.py << 'EOF'
#!/usr/bin/env python3
"""
Demo script for computer vision components
"""

import cv2
import numpy as np
import matplotlib.pyplot as plt

def detect_robots_demo():
    """Demo robot detection using basic computer vision"""
    print("🔍 Computer Vision Demo")
    print("======================")
    
    # Create a simple synthetic image with colored circles (robots)
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    
    # Add some "robots" as colored circles
    robots = [
        (150, 200, (0, 255, 0)),    # Green robot
        (400, 300, (255, 0, 0)),    # Red robot
        (500, 150, (0, 0, 255)),    # Blue robot
    ]
    
    for x, y, color in robots:
        cv2.circle(img, (x, y), 30, color, -1)
        cv2.circle(img, (x, y), 35, (255, 255, 255), 2)
    
    # Convert to grayscale for processing
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Find circles (robots)
    circles = cv2.HoughCircles(
        gray, cv2.HOUGH_GRADIENT, 1, 50,
        param1=50, param2=30, minRadius=20, maxRadius=50
    )
    
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        print(f"Detected {len(circles)} robots!")
        
        for (x, y, r) in circles:
            cv2.circle(img, (x, y), r, (0, 255, 255), 2)
            cv2.putText(img, f"Robot ({x},{y})", (x-40, y-40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # Display result
    plt.figure(figsize=(10, 6))
    plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    plt.title("Robot Detection Demo")
    plt.axis('off')
    plt.show()
    
    print("Demo completed! ✓")

if __name__ == "__main__":
    detect_robots_demo()
EOF

    # Control demo
    cat > src/control/demo.py << 'EOF'
#!/usr/bin/env python3
"""
Demo script for control algorithms
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class SimpleRobot:
    """Simple robot model for demonstration"""
    
    def __init__(self, x=0, y=0, theta=0):
        self.x = x
        self.y = y
        self.theta = theta
        self.history = [(x, y)]
    
    def move(self, v, w, dt=0.1):
        """Move robot with linear velocity v and angular velocity w"""
        self.theta += w * dt
        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt
        self.history.append((self.x, self.y))

def mpc_demo():
    """Demo Model Predictive Control"""
    print("🎮 Control Systems Demo")
    print("=======================")
    
    # Create robots
    leader = SimpleRobot(0, 0, 0)
    follower = SimpleRobot(-2, 0, 0)
    
    # Simulation parameters
    dt = 0.1
    steps = 200
    
    # Leader follows a circular path
    for step in range(steps):
        t = step * dt
        
        # Leader control (circular motion)
        v_leader = 1.0
        w_leader = 0.3
        leader.move(v_leader, w_leader, dt)
        
        # Follower control (simple pursuit)
        dx = leader.x - follower.x
        dy = leader.y - follower.y
        distance = np.sqrt(dx**2 + dy**2)
        
        # Desired distance
        desired_dist = 1.5
        
        # Simple proportional control
        v_follower = 0.5 * (distance - desired_dist)
        angle_to_leader = np.arctan2(dy, dx)
        w_follower = 2.0 * (angle_to_leader - follower.theta)
        
        follower.move(v_follower, w_follower, dt)
    
    # Plot results
    plt.figure(figsize=(12, 8))
    
    leader_x, leader_y = zip(*leader.history)
    follower_x, follower_y = zip(*follower.history)
    
    plt.plot(leader_x, leader_y, 'g-', linewidth=2, label='Leader')
    plt.plot(follower_x, follower_y, 'b-', linewidth=2, label='Follower')
    
    plt.plot(leader_x[0], leader_y[0], 'go', markersize=8, label='Start')
    plt.plot(leader_x[-1], leader_y[-1], 'gs', markersize=8, label='End')
    plt.plot(follower_x[0], follower_y[0], 'bo', markersize=8)
    plt.plot(follower_x[-1], follower_y[-1], 'bs', markersize=8)
    
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.legend()
    plt.title('Leader-Follower Control Demo')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.show()
    
    print("Demo completed! ✓")

if __name__ == "__main__":
    mpc_demo()
EOF

    # Make demo files executable
    chmod +x src/vision/demo.py
    chmod +x src/control/demo.py
    
    print_status "Demo files created ✓"
}

# Create Dockerfile for later ROS2 development
create_dockerfile() {
    print_status "Creating Dockerfile for ROS2 development..."
    
    mkdir -p docker
    
    cat > docker/Dockerfile << 'EOF'
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
COPY requirements.txt /tmp/
RUN pip3 install -r /tmp/requirements.txt

# Set up workspace
WORKDIR /workspace

# Source ROS2 on container start
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

CMD ["bash"]
EOF

    cat > docker/docker-compose.yml << 'EOF'
version: '3.8'

services:
  ros2-dev:
    build:
      context: ..
      dockerfile: docker/Dockerfile
    volumes:
      - ..:/workspace
    environment:
      - DISPLAY=host.docker.internal:0
    ports:
      - "8080:8080"
    stdin_open: true
    tty: true
EOF

    print_status "Docker setup created ✓"
}

# Main execution
main() {
    print_status "Starting macOS project setup..."
    
    check_homebrew
    install_basics
    setup_python_env
    create_demo_files
    create_dockerfile
    
    echo ""
    echo "🎉 macOS Setup Complete!"
    echo "========================"
    echo ""
    echo "To get started:"
    echo "1. source activate_env.sh"
    echo "2. python src/vision/demo.py      # Test computer vision"
    echo "3. python src/control/demo.py     # Test control algorithms"
    echo "4. jupyter lab                    # Start development environment"
    echo ""
    echo "For ROS2 development later:"
    echo "1. Install Docker Desktop"
    echo "2. cd docker && docker-compose up -d"
    echo ""
    echo "Happy coding! 🤖"
}

# Run main function
main "$@" 