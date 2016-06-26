cmake_minimum_required(VERSION 2.8.3)
project(final_approach)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	actionlib_msgs
	actionlib
	sensor_msgs
	message_runtime
	std_msgs
	message_generation
	geometry_msgs
	manager_msg
	ar_track_alvar
	ar_track_alvar_msgs
	trait_im_msg
	nav_msgs
	visualization_msgs
	common_utils
  final_approach_msg
)

# Use clang++-3.6 if available
find_program(CLANGPP clang++-3.6 HINTS /usr/local/bin /usr/bin /opt/local/bin)
if(NOT ${CLANGPP} MATCHES "CLANGPP-NOTFOUND")
	set(CMAKE_CXX_COMPILER ${CLANGPP})
endif(NOT ${CLANGPP} MATCHES "CLANGPP-NOTFOUND")
message(STATUS "Compiler : ${CMAKE_CXX_COMPILER}")
message(STATUS "Found : ${CLANGPP}")

###################################
## catkin specific configuration ##
###################################
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES final_approach
	CATKIN_DEPENDS roscpp rospy actionlib_msgs actionlib sensor_msgs geometry_msgs message_runtime std_msgs message_generation ar_track_alvar nav_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## FinalApproaching_node
add_executable(
	finalApproaching_node

	src/fa_utils.cpp
	src/LaserScan.cpp
	src/Point.cpp
	src/Segment.cpp
	src/FinalApproaching.cpp
	src/Bumperlistener.cpp
	src/ArTagFA.cpp
	src/OdomFA.cpp
	src/Sharps.cpp
	src/GameStateFA.cpp
	src/finalApproaching_node.cpp
)
target_link_libraries(
	finalApproaching_node

	${catkin_LIBRARIES}
)
add_dependencies(
	finalApproaching_node

	${manager_msg_EXPORTED_TARGETS}
	${visualization_msgs_EXPORTED_TARGETS}
	final_approach_msg_gencpp
	comm_msg_gencpp
	manager_msg_gencpp
)
set_target_properties(
	finalApproaching_node

	PROPERTIES COMPILE_FLAGS "-g3 -std=c++11"
)


# Mocked final approach (work on simulation, with mocked machine detection)
add_executable(
	simFinalApproach_node

	src/simFinalApproach_node.cpp
	src/SimFinalApproach.cpp
	src/OdomFA.cpp
)
target_link_libraries(
	simFinalApproach_node

	${catkin_LIBRARIES}
)
add_dependencies(
	simFinalApproach_node

	deplacement_msg_gencpp
	manager_msg_gencpp
)
set_target_properties(
	simFinalApproach_node

	PROPERTIES COMPILE_FLAGS "-g3 -std=c++11")


# Fake_scan_cyril_node
add_executable(
	fake_scan_cyril_node

	src/fake_scan_node.cpp
)
target_link_libraries(
	fake_scan_cyril_node

	${catkin_LIBRARIES}
)
set_target_properties(
	fake_scan_cyril_node

	PROPERTIES COMPILE_FLAG "-g3 -std=c++0x"
)


# AR Tag reader node
add_executable(
	ar_tag_reader_node

	src/ar_tag_reader_node.cpp
)
target_link_libraries(
	ar_tag_reader_node

	${catkin_LIBRARIES}
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark cpp header files for installation
install(
	DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".svn" EXCLUDE
)
