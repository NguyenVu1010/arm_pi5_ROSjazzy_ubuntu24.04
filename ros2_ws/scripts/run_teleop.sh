#!/bin/bash
echo "=== KHOI DONG BAN PHIM (JAZZY) ==="

source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "Su dung cac phim: I, J, K, L de dieu khien"
ros2 run teleop_twist_keyboard teleop_twist_keyboard