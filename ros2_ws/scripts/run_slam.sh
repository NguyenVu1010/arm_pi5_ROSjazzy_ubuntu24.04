#!/bin/bash
echo "=== KHOI DONG SLAM TOOLBOX (JAZZY) ==="

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch my_robot_bringup slam.launch.py