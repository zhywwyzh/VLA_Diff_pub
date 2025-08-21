cmake_minimum_required(VERSION 3.0.2)
project(cloud_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  message_filters
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs pcl_ros pcl_conversions message_filters
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 添加可執行文件
add_executable(pointcloud_filter_node src/pointcloud_filter_node.cpp)

set_target_properties(pointcloud_filter_node PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
)

# 連結庫
target_link_libraries(pointcloud_filter_node
  ${catkin_LIBRARIES}
)

# 添加新的可执行文件
add_executable(timestamp_converter_node src/timestamp_converter_node.cpp)

# 爲此目标链接库
target_link_libraries(timestamp_converter_node
  ${catkin_LIBRARIES}
)