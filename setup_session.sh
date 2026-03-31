#!/bin/bash

echo "Fixing Python setuptools for colcon build..."
pip install setuptools==58.2.0
pip install pyudev

echo "Fetching missing ROS 2 workspace dependencies..."
killall -9 ign ruby rviz2
cd ~/seek_destroy_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

echo "Fetching system updates and installing Nav2..."
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup -y
sudo apt install -y ros-humble-diff-drive-controller ros-humble-joint-state-broadcaster ros-humble-imu-sensor-broadcaster ros-humble-ros2-controllers ros-humble-ign-ros2-control
sudo apt install -y ros-humble-ign-ros2-control ros-humble-ros-ign-bridge ros-humble-ros-ign-gazebo ros-humble-ros-ign-interfaces


echo ""
echo "====================================================="
echo "Environment fixed and dependencies installed!"
echo "The system is fully ready."
echo "Remember to run these in any new terminal you open:"
echo "source /opt/ros/humble/setup.bash"
echo "source ~/seek_destroy_ws/install/setup.bash"
echo "====================================================="