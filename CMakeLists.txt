cmake_minimum_required(VERSION 3.0.2)
project(parallel_link_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES parallel_link_control
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(
  parallel_link_control src/parallel_link_control_node.cpp src/compute_kinematics.cpp src/move_probe.cpp
  )

target_link_libraries(
  parallel_link_control
  ${catkin_LIBRARIES}
)