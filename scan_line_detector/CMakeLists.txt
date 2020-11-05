cmake_minimum_required(VERSION 3.0.2)
project(scan_line_detector)

add_compile_options(-std=c++14 -O3)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  mars_srvs
)

find_package(PCL 1.7 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES scan_line_detector
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs std_msgs tf tf_conversions
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(scan_line_detector_node src/main.cpp 
                                       src/scan_line_detector_node.cpp 
                                       src/laserscan_processor.cpp
                                       src/map_manager.cpp
)

target_link_libraries(scan_line_detector_node ${catkin_LIBRARIES}
                                              ${PCL_LIBRARIES}
) 
