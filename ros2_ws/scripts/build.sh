#!/bin/bash
echo "=== BAT DAU CLEAN VA BUILD (JAZZY) ==="

# 1. Source moi truong he thong
source /opt/ros/jazzy/setup.bash

# 2. Xoa build cu (De tranh xung dot voi Humble cu)
rm -rf build/ install/ log/

# 3. Build
colcon build --symlink-install

# 4. Source lai workspace sau khi build
source install/setup.bash

echo "=== BUILD THANH CONG! ==="