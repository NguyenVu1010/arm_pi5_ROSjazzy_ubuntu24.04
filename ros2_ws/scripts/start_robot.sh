#!/bin/bash
echo "=== KHOI DONG ROBOT (JAZZY) ==="

# 1. Cap quyen USB
echo "[INFO] Cap quyen truy cap cong USB..."
sudo chmod 777 /dev/ttyUSB* 2>/dev/null || true
sudo chmod 777 /dev/ttyACM* 2>/dev/null || true

# 2. Source moi truong
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 3. Chay Launch file
# (Luu y: Code da tu dong chon Real Odom hoac Fake Odom dua tren cau hinh Launch)
ros2 launch my_robot_bringup bringup.launch.py