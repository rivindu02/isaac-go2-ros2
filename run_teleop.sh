#!/bin/bash
# Make sure to source your ROS 2 installation first if not already done in bashrc
# source /opt/ros/humble/setup.bash

echo "Starting teleop_twist_keyboard for Unitree Go2..."
echo "Use I, J, K, L to move around (K to stop)."
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/unitree_go2/cmd_vel
