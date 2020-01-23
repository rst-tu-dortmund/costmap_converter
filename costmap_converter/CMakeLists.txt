cmake_minimum_required(VERSION 3.5)
project(costmap_converter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(class_loader REQUIRED)
find_package(costmap_converter_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

set(dependencies
    class_loader
    costmap_converter_msgs
    cv_bridge
    geometry_msgs
    nav2_costmap_2d
    OpenCV
    pluginlib
    rclcpp
)

include_directories(
  include
)

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

ament_target_dependencies(costmap_converter 
    ${dependencies}
)

add_executable(standalone_converter src/costmap_converter_node.cpp)

target_link_libraries(standalone_converter
    costmap_converter
)
ament_target_dependencies(standalone_converter
  ${dependencies}
)

install(TARGETS costmap_converter
   DESTINATION lib
)
install(TARGETS standalone_converter
   DESTINATION bin
)

install(DIRECTORY include/
   DESTINATION include/
)

install(FILES
  plugins.xml
  DESTINATION share
)

ament_export_include_directories(include)
ament_export_libraries(costmap_converter)
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(costmap_converter plugins.xml)
ament_package()
