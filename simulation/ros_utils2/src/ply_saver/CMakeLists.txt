cmake_minimum_required(VERSION 3.0.2)
project(ply_saver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros pcl_conversions
  DEPENDS PCL
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(save_first_ply src/save_first_ply.cpp)
target_link_libraries(save_first_ply ${catkin_LIBRARIES} ${PCL_LIBRARIES})
