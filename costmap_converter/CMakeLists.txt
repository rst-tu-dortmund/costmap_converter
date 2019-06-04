cmake_minimum_required(VERSION 3.5)
project(costmap_converter)

# Set to Release in order to speed up the program significantly
#set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(costmap_converter_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# TODO : dynamic_reconfigure not supported in ROS2 now
# add dynamic reconfigure api
# find_package(catkin REQUIRED dynamic_reconfigure)
# generate_dynamic_reconfigure_options(
#   cfg/dynamic_reconfigure/CostmapToPolygonsDBSMCCH.cfg
#   cfg/dynamic_reconfigure/CostmapToPolygonsDBSConcaveHull.cfg
#   cfg/dynamic_reconfigure/CostmapToLinesDBSMCCH.cfg
#   cfg/dynamic_reconfigure/CostmapToLinesDBSRANSAC.cfg
#   cfg/dynamic_reconfigure/CostmapToDynamicObstacles.cfg
# )

###########
## Build ##
###########

include_directories(
  include
  ${costmap_converter_msgs_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${nav2_costmap_2d_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${pluginlib_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(costmap_converter SHARED
  src/costmap_to_polygons.cpp 
  src/costmap_to_polygons_concave.cpp
  src/costmap_to_lines_convex_hull.cpp
  src/costmap_to_lines_ransac.cpp
  src/costmap_to_dynamic_obstacles/costmap_to_dynamic_obstacles.cpp
  src/costmap_to_dynamic_obstacles/background_subtractor.cpp
  src/costmap_to_dynamic_obstacles/blob_detector.cpp
  src/costmap_to_dynamic_obstacles/multitarget_tracker/Ctracker.cpp
  src/costmap_to_dynamic_obstacles/multitarget_tracker/Kalman.cpp
  src/costmap_to_dynamic_obstacles/multitarget_tracker/HungarianAlg.cpp
)
set(dependencies
    costmap_converter_msgs
    cv_bridge
    geometry_msgs
    nav2_costmap_2d
    OpenCV
    pluginlib
    rclcpp
    std_msgs
)

ament_target_dependencies(costmap_converter 
    ${dependencies}
)

# Dynamic reconfigure: make sure configure headers raare built before any node using them
#add_dependencies(costmap_converter ${PROJECT_NAME}_gencfg)
# Generate messages before compiling the lib
#add_dependencies(costmap_converter ${PROJECT_NAME}_generate_messages_cpp)

## Declare a cpp executable
add_executable(standalone_converter src/costmap_converter_node.cpp)
target_link_libraries(standalone_converter
    costmap_converter
)
ament_target_dependencies(standalone_converter
  ${dependencies}
)
  
# (un)set: cmake -DCVV_DEBUG_MODE=OFF ..
option(CVV_DEBUG_MODE "cvvisual-debug-mode" ON)
if(CVV_DEBUG_MODE MATCHES ON)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCVVISUAL_DEBUGMODE")
endif()



#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS costmap_converter
   LIBRARY DESTINATION lib
)
install(TARGETS standalone_converter
   RUNTIME DESTINATION bin
)

## Mark cpp header files for installation
install(DIRECTORY include/
   DESTINATION include/
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  plugins.xml
  DESTINATION share
)

ament_export_include_directories(include)
ament_export_libraries(costmap_converter)
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(costmap_converter plugins.xml)
ament_package()
# install(DIRECTORY
#   cfg
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#   PATTERN ".svn" EXCLUDE
# )


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
