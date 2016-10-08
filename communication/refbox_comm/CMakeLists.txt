cmake_minimum_required(VERSION 2.4.6)
project(refbox_comm)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  comm_msg
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#)

## Generate services in the 'srv' folder
#add_service_files(
#  FILES
#)

## Generate added messages and services with any dependencies listed here
 # generate_messages(
 #   DEPENDENCIES
 #   std_msgs
 #   geometry_msgs
 # )
###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
#  DEPENDS system_lib
)

set(LLSF_BASEDIR $ENV{HOME}/refbox/llsf-refbox)
set(CONFDIR ${PROJECT_SOURCE_DIR}/cfg)

# With CMake >= 2.6, got this warning
#   -> Policy CMP0005 is not set: Preprocessor definition values are now escaped automatically
if(POLICY CMP0005)
  cmake_policy(PUSH)
  cmake_policy(SET CMP0005 OLD)
endif()

  add_definitions( -DCONFDIR=\\"${CONFDIR}\\" )
  add_definitions( -DUSERDIR=\\".\\" )

if(POLICY CMP0005)
  cmake_policy(POP)
endif()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${LLSF_BASEDIR}/src
    ${LLSF_BASEDIR}/src/libs
    ${roscpp_INCLUDE_DIRS})

#include_directories(${LLSF_BASEDIR}/src ${roscpp_INCLUDE_DIRS})

link_directories(${LLSF_BASEDIR}/lib)
link_directories(${LLSF_BASEDIR}/lib/protobuf)

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})

add_library(
    refBoxTransport

    src/refBoxTransport.cpp
    src/refBoxMessage.cpp
    src/sendScheduler.cpp
    src/refBoxUtils.cpp
)
add_dependencies(refBoxTransport comm_msg_gencpp)
set_target_properties(refBoxTransport PROPERTIES COMPILE_FLAGS "-std=c++11 -g3")

# See below
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)
# Policy CMP0003 should be set before this line.  Add code such as

#     if(COMMAND cmake_policy)
#       cmake_policy(SET CMP0003 NEW)
#     endif(COMMAND cmake_policy)

#   as early as possible but after the most recent call to
#   cmake_minimum_required or cmake_policy(VERSION).  This warning appears
#   because target "refBoxComm" links to some libraries for which the linker
#   must search:

#     llsf_msgs, llsf_protobuf_comm, protobuf, llsfrbconfig, yaml-cpp

#   and other libraries with known full path:

#     /home/leak/catkin_tests_ws/devel/lib/librefBoxTransport.so
#     /opt/ros/indigo/lib/libroscpp.so

#   CMake is adding directories in the second list to the linker search path in
#   case they are needed to find libraries from the first list (for backwards
#   compatibility with CMake 2.4).  Set policy CMP0003 to OLD or NEW to enable
#   or disable this behavior explicitly.  Run "cmake --help-policy CMP0003" for
#   more information.

add_executable(
  refBoxComm

  src/refBoxComm.cpp
  src/refBoxComm_node.cpp
)

target_link_libraries(
  refBoxComm

  refBoxTransport
  llsf_msgs
  llsf_protobuf_comm
  protobuf
  llsfrbconfig
  #llsfrbcore
  #llsfrbutils
  yaml-cpp

  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
)

add_dependencies(refBoxComm ${catkin_EXPORTED_TARGETS} comm_msg_gencpp)
set_target_properties(refBoxComm PROPERTIES COMPILE_FLAGS "-std=c++11 -g3")


add_executable(
  eiListener
  src/explorationInfoListenerNode.cpp
)

target_link_libraries(
  eiListener
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
)

add_dependencies(eiListener ${catkin_EXPORTED_TARGETS} comm_msg_gencpp)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark executable scripts (Python etc.) for installation
# in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/test_explorationInfoListener.py
  scripts/test_gameStateListener.py
  scripts/test_reportMachine.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark executables and/or libraries for installation
install(TARGETS refBoxComm eiListener
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
