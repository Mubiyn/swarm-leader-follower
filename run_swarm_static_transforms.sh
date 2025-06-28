#!/bin/bash
# Run all static transforms for swarm robots
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 map leader/base_link &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 map follower_1/base_link &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 map follower_2/base_link &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 map follower_3/base_link &
wait 