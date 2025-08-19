cmake_minimum_required(VERSION 3.0.2)
project(goal_converter)

## 编译选项
add_compile_options(-std=c++11)

## 依赖的 ROS 包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  quadrotor_msgs
)

## catkin 配置
catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs quadrotor_msgs
)

## 头文件路径
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## 生成可执行文件
add_executable(goal_converter_node src/goal_converter.cpp)

## 链接 ROS 库
target_link_libraries(goal_converter_node
  ${catkin_LIBRARIES}
)
