cmake_minimum_required(VERSION 2.8.3)
project(gazebo_to_ros)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    gazebo_ros
    message_generation
    trait_im_msg
    nav_msgs
    deplacement_msg
    gazebo_msgs
    tf
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

set(GAZEBO_RCLL_BASEDIR $ENV{HOME}/gazebo-rcll)

###################################
## catkin specific configuration ##
###################################

catkin_package(
	# INCLUDE_DIRS include
    #  LIBRARIES robotino3_gazebo
    CATKIN_DEPENDS message_runtime
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
    ${GAZEBO_RCLL_BASEDIR}/plugins/src/libs/
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

link_directories(
	${GAZEBO_RCLL_BASEDIR}/plugins/lib/protobuf/
)

## Declare a cpp executable
add_executable(gazebo_to_ros_node
    src/gazebo_to_ros_node.cpp
)
target_link_libraries(gazebo_to_ros_node
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    ${Boost_LIBRARIES}
    libgazsim_msgs.so.2014
)
add_dependencies(gazebo_to_ros_node
	trait_im_msg_gencpp
	deplacement_msg_gencpp
)
#set_target_properties(gazebo_to_ros_node PROPERTIES COMPILE_FLAGS "-g3")
