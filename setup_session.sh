#!/bin/bash

echo "Fixing Python setuptools for colcon build..."
pip install setuptools==58.2.0
pip install pyudev
pip install transforms3d

echo "Fetching missing ROS 2 workspace dependencies..."
killall -9 ign ruby rviz2
cd ~/seek_destroy_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

echo "Fetching system updates and installing Nav2..."
sudo apt update

# Consolidated all apt packages into one command for a faster install
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-nav2-msgs \
  ros-humble-cv-bridge \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-diff-drive-controller \
  ros-humble-joint-state-broadcaster \
  ros-humble-imu-sensor-broadcaster \
  ros-humble-ros2-controllers \
  ros-humble-ign-ros2-control \
  ros-humble-ros-ign-bridge \
  ros-humble-ros-ign-gazebo \
  ros-humble-ros-ign-interfaces \
  ros-humble-nav2-lifecycle-manager \
  python3-opencv \
  python3-yaml

echo ""
echo "====================================================="
echo "Environment fixed and dependencies installed!"
echo "The system is fully ready."
echo "Remember to run these in any new terminal you open:"
echo "source /opt/ros/humble/setup.bash"
echo "source ~/seek_destroy_ws/install/setup.bash"
echo "====================================================="