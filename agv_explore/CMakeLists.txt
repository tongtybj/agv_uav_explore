cmake_minimum_required(VERSION 3.0.2)
project(agv_explore)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
#catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   PositionAndGas.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy tf message_runtime sensor_msgs std_msgs
)

## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(existing_path_generator src/existing_path_generator.cpp)
target_link_libraries (existing_path_generator ${catkin_LIBRARIES})

add_library(existing_path_global_planner src/existing_path_global_planner.cpp)
target_link_libraries(existing_path_global_planner ${catkin_LIBRARIES})

add_executable(existing_path_global_planner_test src/existing_path_global_planner_test.cpp)
target_link_libraries (existing_path_global_planner_test existing_path_global_planner ${catkin_LIBRARIES})



catkin_install_python(PROGRAMS
  scripts/simple_explore.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

