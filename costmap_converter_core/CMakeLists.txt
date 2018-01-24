cmake_minimum_required(VERSION 2.8.3)
project(costmap_converter_core)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  costmap_converter_msgs
  nav_msgs
  pluginlib
  roscpp
  visualization_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES 
  CATKIN_DEPENDS
    costmap_2d
    costmap_converter_msgs
    nav_msgs
    pluginlib
    roscpp
    visualization_msgs
  #DEPENDS
)

###########
## Build ##
###########

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(costmap_converter_node src/costmap_converter_node.cpp)
target_link_libraries(costmap_converter_node ${catkin_LIBRARIES})

# Set required C++11 flag
if(CMAKE_VERSION VERSION_LESS "3.1")
  set_target_properties(costmap_converter_node PROPERTIES COMPILE_FLAGS "-std=c++11")
else()
  set_property(TARGET costmap_converter_node PROPERTY CXX_STANDARD 11)
  set_property(TARGET costmap_converter_node PROPERTY CXX_STANDARD_REQUIRED ON)
  set_property(TARGET costmap_converter_node PROPERTY CXX_EXTENSIONS OFF)
endif()

# (un)set: cmake -DCVV_DEBUG_MODE=OFF ..
option(CVV_DEBUG_MODE "cvvisual-debug-mode" ON)
if(CVV_DEBUG_MODE MATCHES ON)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCVVISUAL_DEBUGMODE")
endif()

#############
## Install ##
#############

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME}
  #  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )
install(TARGETS costmap_converter_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   #FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_drawing.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
