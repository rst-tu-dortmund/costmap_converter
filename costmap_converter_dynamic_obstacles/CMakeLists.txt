cmake_minimum_required(VERSION 2.8.3)
project(costmap_converter_dynamic_obstacles)

find_package(catkin REQUIRED COMPONENTS
  costmap_converter_core
  dynamic_reconfigure
  cv_bridge
)

# find_package(OpenCV REQUIRED)

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/dynamic_reconfigure/CostmapToDynamicObstacles.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    costmap_converter_core
    dynamic_reconfigure
    cv_bridge
  # DEPENDS
    # OpenCV
)

###########
## Build ##
###########

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
#   # ${OpenCV_INCLUDE_DIRS}
# )

## Declare a cpp library
add_library(${PROJECT_NAME}
  src/costmap_to_dynamic_obstacles.cpp
  src/background_subtractor.cpp
  src/blob_detector.cpp
  src/multitarget_tracker/Ctracker.cpp
  src/multitarget_tracker/Kalman.cpp
  src/multitarget_tracker/HungarianAlg.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}
# ${OpenCV_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

# Set required C++11 flag
if(CMAKE_VERSION VERSION_LESS "3.1")
  set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-std=c++11")
else()
  set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
  set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)
  set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_EXTENSIONS OFF)
endif()

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   #FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
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
