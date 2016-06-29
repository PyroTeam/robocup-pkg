cmake_minimum_required(VERSION 2.8.3)
project(feu_tricolore)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  trait_im_msg
  image_transport
  cv_bridge
  actionlib
  common_utils
)

find_package(OpenCV)

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
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES feu_tricolore
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

# Declare all cpp executables
add_executable(
	light_signal_detection_node

	src/lecture_feu_node.cpp
	src/cv_algo.cpp
	src/LightDetection.cpp
	src/display-histogram.cpp
)
add_dependencies(
	light_signal_detection_node

	trait_im_msg_generate_messages_cpp
)
target_link_libraries(
	light_signal_detection_node

	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
)
set_target_properties(
	light_signal_detection_node

	PROPERTIES COMPILE_FLAGS "-std=c++11 -Wall -Werror"	# Uniquement en dev sur machine perso, pas en release
	# PROPERTIES COMPILE_FLAGS "-std=c++11 -Wall"
)

add_executable(sim_light_detection
	src/simLightDetection_node.cpp
	src/SimLightDetection.cpp
)
add_dependencies(sim_light_detection
	trait_im_msg_gencpp
)
target_link_libraries(sim_light_detection
	${catkin_LIBRARIES}
)