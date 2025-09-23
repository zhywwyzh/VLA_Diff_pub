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

# ======== 新增：可选 CUDA =========
include(CheckLanguage)
check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
  enable_language(CUDA)
  message(STATUS "[ply_publisher] CUDA enabled: ${CMAKE_CUDA_COMPILER}")
  set(HAVE_CUDA TRUE)
else()
  message(WARNING "[ply_publisher] CUDA compiler not found; will build CPU-only")
  set(HAVE_CUDA FALSE)
endif()
# =================================

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

# ======== 新增：渲染器库 =========
# 需要把下述三个文件加入工程：
# include/ply_publisher/depth_renderer.h
# src/depth_renderer_cpu.cpp
# src/depth_renderer.cu
add_library(depth_renderer_cpu
  src/depth_renderer_cpu.cpp
)
target_link_libraries(depth_renderer_cpu
  ${catkin_LIBRARIES}
)

if(HAVE_CUDA)
  add_library(depth_renderer_cuda
    src/depth_renderer.cu
  )
  # 依据你的 GPU 调整算力列表
  set_target_properties(depth_renderer_cuda PROPERTIES
    CUDA_ARCHITECTURES "75;80;86;89"
  )
  target_link_libraries(depth_renderer_cuda
    ${catkin_LIBRARIES}
  )
endif()
# =================================

add_executable(ply_publisher_node
  src/pc_processor.cpp
  src/sensor_manager.cpp
  src/ground_align.cpp
  src/ply_publisher_node.cpp
)

target_link_libraries(ply_publisher_node
  depth_renderer_cpu
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  Eigen3::Eigen
)

# 链接 CUDA 版（若可用）
if(HAVE_CUDA)
  target_link_libraries(ply_publisher_node depth_renderer_cuda)
  target_compile_definitions(ply_publisher_node PRIVATE USES_OPTIONAL_CUDA=1)
endif()

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
