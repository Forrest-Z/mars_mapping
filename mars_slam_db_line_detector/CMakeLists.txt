cmake_minimum_required(VERSION 3.0.2)
project(mars_slam_db_line_detector)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14 -O3)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  tf
  mars_srvs
  tf_conversions
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mars_slam_db_line_detector
#  CATKIN_DEPENDS nav_msgs roscpp tf
#  DEPENDS system_lib
)


include_directories(
  include/mars_slam_db_line_detector
  ${catkin_INCLUDE_DIRS}
)


add_executable(db_line_detector_node src/main.cpp 
                                     src/db_line_detector_node.cpp
                                     src/map_manager.cpp
)

target_link_libraries(db_line_detector_node ${catkin_LIBRARIES}
) 
