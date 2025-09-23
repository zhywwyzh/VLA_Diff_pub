cmake_minimum_required(VERSION 3.0.2)
project(ply_publisher)

set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  cv_bridge
  image_transport
)

find_package(PCL REQUIRED COMPONENTS common io filters segmentation)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs pcl_ros pcl_conversions cv_bridge image_transport
  DEPENDS PCL EIGEN3 OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ply_publisher_node
  src/pc_processor.cpp
  src/sensor_manager.cpp
  src/ground_align.cpp
  src/ply_publisher_node.cpp
)

target_link_libraries(ply_publisher_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  Eigen3::Eigen
)

install(TARGETS ply_publisher_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
