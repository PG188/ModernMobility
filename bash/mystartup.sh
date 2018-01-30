#!/bin/bash
cd ~/catkin_ws
echo "compiling ROS packages and nodes..."
catkin_make
echo "done compiling."
source devel/setup.bash
echo "launching overlord..."
roslaunch overlord overlord.launch
