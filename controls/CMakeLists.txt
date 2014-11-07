cmake_minimum_required(VERSION 2.8.3)
project(controls)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  cv_bridge
  message_generation
  robosub_msg
  planner
  tf
)

################################################
## Declare ROS messages, services and actions ##
################################################


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  motorCommands.msg
  VectorPID.msg
  DebugControls.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

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
#  LIBRARIES controls
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs robosub_msg message_runtime planner
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)




## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

## copied from writingPublisherSubscriber tutorial
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(depthController src/depthController.cpp)
add_executable(controls src/controls.cpp)
add_executable(thrust_mapper src/thrust_mapper.cpp)
add_executable(gui_controls src/gui_controls.cpp)


target_link_libraries(controls ${catkin_LIBRARIES})
target_link_libraries(depthController ${catkin_LIBRARIES})
target_link_libraries(thrust_mapper ${catkin_LIBRARIES})
target_link_libraries(gui_controls ${catkin_LIBRARIES})


add_dependencies(depthController ${catkin_EXPORTED_TARGETS})
add_dependencies(controls ${catkin_EXPORTED_TARGETS} controls_generate_messages_cpp)
add_dependencies(thrust_mapper ${catkin_EXPORTED_TARGETS} controls_generate_messages_cpp)
add_dependencies(gui_controls ${catkin_EXPORTED_TARGETS} controls_generate_messages_cpp)


#add_dependencies(controls ${PROJECT_NAME}_gencpp)

#add_library(thrust_mapper_lib src/thrust_mapper.cpp)

add_library(controls_lib src/controls.cpp)
target_link_libraries(controls_lib ${catkin_LIBRARIES})
add_dependencies(controls_lib ${catkin_EXPORTED_TARGETS})

catkin_add_gtest(${PROJECT_NAME}-test  tests/test_controls.cpp)

target_link_libraries(${PROJECT_NAME}-test 
  ${catkin_LIBRARIES}
  controls_lib
)

