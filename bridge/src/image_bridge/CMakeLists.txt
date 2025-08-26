cmake_minimum_required(VERSION 3.0.2)
project(image_bridge)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES image_bridge
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(image_bridge_node 
  src/image_bridge_node.cpp
)
target_link_libraries(image_bridge_node
  ${catkin_LIBRARIES}
)

install(TARGETS image_bridge_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
