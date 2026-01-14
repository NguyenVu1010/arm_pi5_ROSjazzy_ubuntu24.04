#!/bin/bash

echo "========================================"
echo "      LUU BAN DO VAO THU MUC MAPS       "
echo "========================================"

# 1. Source moi truong ROS 2
# Tu dong phat hien Jazzy hoac Humble
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

source ~/phong_le_10_1/ros2_ws/install/setup.bash

# 2. Dinh nghia duong dan tuyet doi den thu muc maps
# (Dung $HOME de thay cho /home/nguyen1, dam bao chay dung tren moi user)
WORKSPACE_DIR="$HOME/phong_le_10_1/ros2_ws"
MAP_DIR="$WORKSPACE_DIR/maps"

# 3. Tao thu muc maps neu chua co
if [ ! -d "$MAP_DIR" ]; then
    echo "[INFO] Thu muc maps chua ton tai. Dang tao moi..."
    mkdir -p "$MAP_DIR"
fi

# 4. Xu ly ten Ban do
if [ -z "$1" ]; then
    # Neu khong nhap ten -> Dat theo ngay gio (VD: map_20240114_103000)
    MAP_NAME="map_$(date +%Y%m%d_%H%M%S)"
    echo "[INFO] Ban khong dat ten. Tu dong dat la: $MAP_NAME"
else
    # Neu co nhap ten
    MAP_NAME=$1
fi

# 5. Duong dan day du cua file se luu
FULL_PATH="$MAP_DIR/$MAP_NAME"

echo "[EXEC] Dang luu ban do tai: $FULL_PATH"

# 6. Goi lenh luu map
ros2 run nav2_map_server map_saver_cli -f "$FULL_PATH"

echo "========================================"
if [ -f "${FULL_PATH}.yaml" ]; then
    echo "[THANH CONG] Da luu 2 file:"
    echo "  1. ${FULL_PATH}.pgm"
    echo "  2. ${FULL_PATH}.yaml"
else
    echo "[THAT BAI] Khong the luu map. Kiem tra lai SLAM Toolbox."
fi
echo "========================================"