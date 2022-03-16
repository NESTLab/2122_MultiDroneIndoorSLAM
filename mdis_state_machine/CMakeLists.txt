cmake_minimum_required(VERSION 2.8.3)
project(mdis_state_machine)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib_msgs
  actionlib
  move_base_msgs
  genmsg
  sensor_msgs
  geometry_msgs
  message_generation
  octomap_msgs
  nav_msgs
  tf
)

catkin_python_setup()

add_message_files(
  FILES
  DataCommunication.msg
  RobotsState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    actionlib_msgs
    message_runtime
    actionlib
    move_base_msgs
    genmsg
    sensor_msgs
    geometry_msgs
    octomap_msgs
    nav_msgs
    tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

###############
## Libraries ##
###############

## Declare a C++ library
add_library(${PROJECT_NAME}
src/move_base_interface.cpp
src/robot_state_machine.cpp
src/team_scheduler.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


## Test Move base
add_executable(move_base_test src/move_base_test.cpp)
add_dependencies(move_base_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_base_test ${PROJECT_NAME} ${catkin_LIBRARIES})

## Test Move base
add_executable(team_scheduler src/team_scheduler.cpp)
add_dependencies(team_scheduler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(team_scheduler ${PROJECT_NAME} ${catkin_LIBRARIES})

## Test Move base

install(TARGETS move_base_test
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
