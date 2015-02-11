cmake_minimum_required(VERSION 2.4.6)
project(refBoxComm)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
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
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )
###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
#  DEPENDS system_lib
)

#set(LLSF_BASEDIR /home/vincent/refbox/llsf-refbox-crypto)
set(LLSF_BASEDIR $ENV{HOME}/refbox/llsf-refbox)
#set(CONFDIR ${LLSF_BASEDIR}/cfg)
set(CONFDIR ${PROJECT_SOURCE_DIR}/cfg)

add_definitions( -DCONFDIR=\\"${CONFDIR}\\" )
add_definitions( -DUSERDIR=\\".\\" )

include_directories(${LLSF_BASEDIR}/src/libs ${roscpp_INCLUDE_DIRS})
include_directories(${LLSF_BASEDIR}/src ${roscpp_INCLUDE_DIRS})

link_directories(${LLSF_BASEDIR}/lib)
link_directories(${LLSF_BASEDIR}/lib/protobuf)

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})

add_library(refBoxTransport src/refBoxTransport.cpp)

add_executable(
  refBoxComm 
  src/refBoxCommNode.cpp 
)

add_dependencies(refBoxComm ${catkin_EXPORTED_TARGETS})

add_executable(
  eiListener
  src/explorationInfoListenerNode.cpp 
)

add_dependencies(eiListener ${catkin_EXPORTED_TARGETS})


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

target_link_libraries( 
  eiListener
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
)

set_target_properties(refBoxTransport PROPERTIES COMPILE_FLAGS "-std=c++0x -g3")
set_target_properties(refBoxComm PROPERTIES COMPILE_FLAGS "-std=c++0x -g3")

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

