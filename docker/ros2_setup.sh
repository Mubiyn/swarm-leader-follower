#!/bin/bash

# ROS2 Docker Setup for Modern Swarm System
echo "🐳 Setting up ROS2 in Docker for Modern Swarm System"

# Pull ROS2 Humble image
docker pull osrf/ros:humble-desktop

# Create Docker network for multi-robot simulation
docker network create swarm_network 2>/dev/null || true

# Run ROS2 container with GUI support (macOS)
docker run -it --rm \
    --name ros2_swarm \
    --network swarm_network \
    -v "$(pwd)":/workspace \
    -w /workspace \
    -e DISPLAY=host.docker.internal:0 \
    osrf/ros:humble-desktop \
    bash -c "
        source /opt/ros/humble/setup.bash && \
        echo '🚀 ROS2 Ready! Now you can run:' && \
        echo '  ros2 launch launch/fast_track_migration.launch.py' && \
        echo '  python3 quick_start_migration.py --sim' && \
        bash
    " 