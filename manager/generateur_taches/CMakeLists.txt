cmake_minimum_required(VERSION 2.8.3)
project(generateur_taches)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  manager_msg
  common_utils
  geometry_utils
  deplacement_msg
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
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service.srv
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
#   std_msgs
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
#  LIBRARIES generateur_taches
  CATKIN_DEPENDS roscpp rospy std_msgs deplacement_msg
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
  include/generateur_taches
)

## Declare a cpp library
# add_library(generateur_taches
#   src/${PROJECT_NAME}/generateur_taches.cpp
# )

## Declare a cpp executable
#add_executable(work_fonctions_test
#  src/task.cpp
#  src/tasksList.cpp
#  src/work.cpp
#  src/product.cpp
#  src/workFonctionsTest_node.cpp)
#target_link_libraries(work_fonctions_test ${catkin_LIBRARIES})
#add_dependencies(work_fonctions_test manager_msg_gencpp)
#set_target_properties(work_fonctions_test PROPERTIES COMPILE_FLAGS "-g3 -std=c++0x")


add_executable(main_node
  src/gameState.cpp
  src/workPerPhase.cpp
  src/tabmachine.cpp
  src/machine.cpp
  src/storage.cpp
  src/storageChart.cpp
  src/srvorder.cpp
  src/action.cpp
  src/task.cpp
  src/tasksList.cpp
  src/work.cpp
  src/product.cpp
  src/correspondanceZE.cpp
  src/orderInfo.cpp
  src/robotInfo.cpp
  src/robot.cpp
  src/LocaSubscriber.cpp
  src/main_node.cpp)
target_link_libraries(main_node ${catkin_LIBRARIES} )
add_dependencies(main_node manager_msg_gencpp comm_msg_gencpp)
set_target_properties(main_node PROPERTIES COMPILE_FLAGS "-g3 -std=c++0x")

add_executable(orderInfo_node
    src/orderInfo.cpp
    src/orderInfo_node.cpp)
target_link_libraries(orderInfo_node ${catkin_LIBRARIES} )
add_dependencies(orderInfo_node comm_msg_gencpp)
set_target_properties(orderInfo_node PROPERTIES COMPILE_FLAGS "-g3 -std=c++0x")

add_executable(robotInfo_node
    src/robotInfo.cpp
    src/robotInfo_node.cpp)
target_link_libraries(robotInfo_node ${catkin_LIBRARIES} )
add_dependencies(robotInfo_node comm_msg_gencpp)
set_target_properties(robotInfo_node PROPERTIES COMPILE_FLAGS "-g3 -std=c++0x")

add_executable(action_node
    src/action.cpp
    src/action_node.cpp)
target_link_libraries(action_node ${catkin_LIBRARIES} )
add_dependencies(action_node manager_msg_gencpp)
set_target_properties(action_node PROPERTIES COMPILE_FLAGS "-g3 -std=c++0x")


add_executable(sendOrder_node
	src/sendOrder_node.cpp)
target_link_libraries(sendOrder_node ${catkin_LIBRARIES})
add_dependencies(sendOrder_node manager_msg_gencpp)
set_target_properties(sendOrder_node PROPERTIES COMPILE_FLAGS "-g3 -std=c++0x")


add_executable(orderListener_node
	src/orderListener_node.cpp)
target_link_libraries(orderListener_node ${catkin_LIBRARIES})
add_dependencies(orderListener_node manager_msg_gencpp)
set_target_properties(orderListener_node PROPERTIES COMPILE_FLAGS "-g3 -std=c++0x")


add_executable(activityTalker_node
	src/activityTalker_node.cpp)
target_link_libraries(activityTalker_node ${catkin_LIBRARIES})
add_dependencies(activityTalker_node manager_msg_gencpp)
set_target_properties(activityTalker_node PROPERTIES COMPILE_FLAGS "-g3 -std=c++0x")


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(generateur_taches_node generateur_taches_generate_messages_cpp)

## Specify libraries to link a library or executable target against

add_library(manager_utils
    src/gameState.cpp
)
add_dependencies(manager_utils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

set_target_properties(manager_utils PROPERTIES COMPILE_FLAGS "-std=c++11")
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
# install(TARGETS generateur_taches generateur_taches_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_generateur_taches.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
