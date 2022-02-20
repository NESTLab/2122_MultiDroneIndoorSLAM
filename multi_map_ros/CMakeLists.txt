cmake_minimum_required(VERSION 2.8.3)
project(multi_map_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    dag_node.msg
    )

## Generate services in the 'srv' folder
add_service_files(
   FILES
   merge_srv.srv
   )


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  )
