#!/bin/bash
echo "=== KHOI DONG NAVIGATION JAZZY ==="

source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Dinh nghia thu muc map
WORKSPACE_DIR="$HOME/phong_le_10_1/ros2_ws"
MAP_DIR="$WORKSPACE_DIR/maps"

# Lay ten map tu tham so, mac dinh la 'my_map'
MAP_NAME=${1:-my_map}
MAP_FILE="$MAP_DIR/$MAP_NAME.yaml"

if [ ! -f "$MAP_FILE" ]; then
    echo "[LOI] Khong tim thay file map: $MAP_FILE"
    echo "Hay kiem tra trong thu muc: $MAP_DIR"
    exit 1
fi

echo "[INFO] Dang tai ban do: $MAP_FILE"

# Chay Nav2 voi map cu the
ros2 launch my_robot_bringup nav.launch.py map:="$MAP_FILE"