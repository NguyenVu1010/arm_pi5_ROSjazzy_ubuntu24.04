cmake_minimum_required(VERSION 3.8)
project(arm_slam)

find_package(ament_cmake REQUIRED)

# Cài đặt thư mục config để slam_toolbox đọc được file .yaml
install(
  DIRECTORY config #launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()