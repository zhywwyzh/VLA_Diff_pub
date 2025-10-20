#!/bin/bash

cd /home/zhywwyzh/workspace/VLA_Diff
unset ROS_DISTRO
unset ROS_VERSION
unset ROS_PYTHON_VERSION
unset ROS_ROOT
unset ROS_PACKAGE_PATH
unset ROS_MASTER_URI
unset ROS_IP
unset ROS_HOSTNAME
unset CMAKE_PREFIX_PATH
unset LD_LIBRARY_PATH
unset PYTHONPATH
unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH
source /opt/ros/noetic/setup.bash
source /home/zhywwyzh/workspace/VLA_Diff/Openpi/.venv/bin/activate
source /home/zhywwyzh/workspace/VLA_Diff/simulation/ros_utils2/devel/setup.bash
