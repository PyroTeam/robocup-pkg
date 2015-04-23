cmake_minimum_required(VERSION 2.8.3)
project(executeur_taches)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  genmsg actionlib_msgs
  actionlib
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Landmarks.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

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
  INCLUDE_DIRS include
#  LIBRARIES executeur_taches
  CATKIN_DEPENDS roscpp rospy actionlib_msgs actionlib
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## Declare a cpp library
# add_library(executeur_taches
#   src/${PROJECT_NAME}/executeur_taches.cpp
# )

## Declare a cpp executable
add_executable(
  test_exec
  
  src/main.cpp
  src/Machine.cpp
  src/BaseStation.cpp
  src/CapStation.cpp
  src/RingStation.cpp
  src/DeliveryStation.cpp
  src/GtServerSrv.cpp
  src/LocaSubscriber.cpp
  src/GripperClientSrv.cpp
  src/MyElements.cpp
  src/FeuClientAction.cpp
  src/FinalApproachingClient.cpp
  src/ExploInfoSubscriber.cpp
  src/ReportingMachineSrvClient.cpp
  src/ArTagClientSrv.cpp
)
add_executable(
  test_publish
  
  src/fake_publisher_node.cpp
)
add_executable(
  test_lights
  
  src/fake_light_server_node.cpp
)
add_executable(
  test_order
  
  src/fake_client_order.cpp
)
add_executable(
  test_ei
  
  src/fake_publisher_ei.cpp
)
add_executable(
  test_arTag
  
  src/fake_arTag_server.cpp
)
add_executable(
  test_report
  
  src/fake_server_reporting.cpp
)
##set_target_properties(test_exec PROPERTIES COMPILE_FLAGS -std=c++0x) # c++ version 11
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(executeur_taches_node executeur_taches_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(
  test_exec
  ${catkin_LIBRARIES}
)
target_link_libraries(
  test_publish
  ${catkin_LIBRARIES}
)
target_link_libraries(
  test_lights
  ${catkin_LIBRARIES}
)
target_link_libraries(
  test_order
  ${catkin_LIBRARIES}
)
target_link_libraries(
  test_ei
  ${catkin_LIBRARIES}
)
target_link_libraries(
  test_arTag
  ${catkin_LIBRARIES}
)
target_link_libraries(
  test_report
  ${catkin_LIBRARIES}
)
#target_link_libraries(
#  test_topic
#  ${catkin_LIBRARIES}
#)
add_dependencies(
  test_exec
  ${manager_msg_EXPORTED_TARGETS}
  manager_msg_gencpp
)
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS executeur_taches executeur_taches_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_executeur_taches.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
