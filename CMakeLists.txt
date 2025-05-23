cmake_minimum_required(VERSION 3.5)
project(gemini335l)

# コンパイラの設定
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# パッケージ依存
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# インクルードディレクトリの設定
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# depth_to_pointcloud_node をビルド
add_executable(depth_to_pointcloud_node src/point_cloud.cpp)

ament_target_dependencies(depth_to_pointcloud_node
  rclcpp
  sensor_msgs
  cv_bridge
  message_filters
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  OpenCV
)

# インストール設定
install(TARGETS
  depth_to_pointcloud_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# パッケージ登録
ament_package()

