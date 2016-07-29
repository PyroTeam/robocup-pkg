cmake_minimum_required(VERSION 2.8.3)
project(localisation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  visualization_msgs
  cmake_modules
  deplacement_msg
  tf
  message_filters
    geometry_utils
    comm_msg
    common_utils
    ar_track_alvar
    ar_track_alvar_msgs
)

## System dependencies are found with CMake's conventions
find_package(Eigen REQUIRED)


###################################
## catkin specific configuration ##
###################################

catkin_package(
   INCLUDE_DIRS include/localisation /usr/include/eigen3
   LIBRARIES localisation
   CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs message_runtime geometry_utils comm_msg common_utils ar_track_alvar
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  include/localisation
)

## Declare a cpp executable
add_executable(
  landmarks_extraction
  src/landmarks_extraction_node.cpp
  src/landmarks_detection_utils.cpp
  src/math_functions.cpp
  src/conversion_functions.cpp
  src/LaserScan.cpp
  src/Line.cpp
  src/Segment.cpp
  src/Model.cpp
  src/Machine.cpp)
set_target_properties(landmarks_extraction PROPERTIES COMPILE_FLAGS "-g3 -std=c++11")
add_dependencies(landmarks_extraction deplacement_msg_gencpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  landmarks_extraction
  ${catkin_LIBRARIES}
)

add_executable(
  odometry_correction
  src/odometry_correction_node.cpp)
set_target_properties(odometry_correction PROPERTIES COMPILE_FLAGS "-g3 -std=c++11")
add_dependencies(odometry_correction deplacement_msg_gencpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  odometry_correction
  ${catkin_LIBRARIES}
)

add_executable(
  visualisation
  src/visualisation.cpp)
set_target_properties(visualisation PROPERTIES COMPILE_FLAGS "-g3 -std=c++11")
add_dependencies(visualisation deplacement_msg_gencpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  visualisation
  ${catkin_LIBRARIES}
)

add_executable(
  cartographie
  src/cartographie_node.cpp
  src/landmarks_detection_utils.cpp
  src/math_functions.cpp
  src/conversion_functions.cpp
  src/Segment.cpp
  src/Machine.cpp
  src/Line.cpp
  src/Model.cpp)
set_target_properties(cartographie PROPERTIES COMPILE_FLAGS "-g3 -std=c++11")
add_dependencies(cartographie deplacement_msg_gencpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  cartographie
  ${catkin_LIBRARIES}
)

add_executable(
  ekf
  src/ekf_node.cpp
  src/Ekf.cpp
  src/LandmarksDetectionSubscriber.cpp)
set_target_properties(ekf PROPERTIES COMPILE_FLAGS "-g3 -std=c++11")
add_dependencies(ekf deplacement_msg_gencpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  ekf
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
