cmake_minimum_required(VERSION 2.6.3)
project(itasc_robot_pr2_test)

set (ROS_ROOT $ENV{ROS_ROOT} )
if (ROS_ROOT)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  rosbuild_init()
  rosbuild_find_ros_package( rtt )
  set( RTT_HINTS HINTS ${rtt_PACKAGE_PATH}/../install )
endif()
find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})

include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

orocos_component(itasc_robot_pr2_test src/itasc_robot_pr2_test.cpp) # ...you may add multiple source files
orocos_install_headers( src/itasc_robot_pr2_test.hpp ) # ...you may add multiple header files
orocos_generate_package()
