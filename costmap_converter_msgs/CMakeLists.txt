cmake_minimum_required(VERSION 2.8.3)
project(costmap_converter_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  # message_runtime
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ObstacleMsg.msg
  ObstacleArrayMsg.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES costmap_converter_core
  CATKIN_DEPENDS
    geometry_msgs
    std_msgs
    message_runtime
  #DEPENDS
)
