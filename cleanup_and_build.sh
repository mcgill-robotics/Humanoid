#!/bin/bash

cd ~/Humanoid/catkin_ws/ || { echo "Failed to navigate to catkin_ws"; exit 1; }

echo "Cleaning catkin build..."
catkin clean -b --yes || { echo "Failed to clean catkin build"; exit 1; }

cd src/xsens_ros_mti_driver/lib/xspublic/ || { echo "Failed to navigate to xsens_ros_mti_driver lib/xspublic"; exit 1; }

echo "Cleaning xsens_ros_mti_driver make files..."
make clean || { echo "Failed to clean xsens_ros_mti_driver"; exit 1; }

cd ~/Humanoid/catkin_ws/src/ || { echo "Failed to navigate back to src"; exit 1; }

echo "Changing permissions for xsens_ros_mti_driver..."
sudo chmod o+rw xsens_ros_mti_driver/ || { echo "Failed to change permissions for xsens_ros_mti_driver"; exit 1; }

echo "Building xsens_ros_mti_driver..."
pushd xsens_ros_mti_driver/lib/xspublic && make && popd || { echo "Failed to build xsens_ros_mti_driver"; exit 1; }

cd ~/Humanoid/catkin_ws/src/ || { echo "Failed to navigate to catkin_ws/src"; exit 1; }
echo "Building the catkin workspace..."
catkin build || { echo "Failed to build the catkin workspace"; exit 1; }

echo "Build and cleanup completed successfully."
