cmake_minimum_required(VERSION 2.8.3)
project(gazebo_to_ros)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  nav_msgs
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robotino3_gazebo
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${GAZEBO_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(gazebo_to_ros_node
    src/gazebo_to_ros_node.cpp
)
target_link_libraries(gazebo_to_ros_node
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
)
