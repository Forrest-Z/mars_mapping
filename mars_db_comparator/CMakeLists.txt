cmake_minimum_required(VERSION 3.0.2)
project(mars_db_comparator)

add_compile_options(-std=c++14 -O3)

find_package(catkin REQUIRED COMPONENTS
  mars_srvs
  nav_msgs
  roscpp
  tf
  tf_conversions
)

find_package(PCL 1.7 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mars_db_comparator
#  CATKIN_DEPENDS mars_slam_srvs nav_msgs roscpp tf
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(db_comparator_node src/main.cpp 
                                  src/db_comparator_node.cpp 
                                  src/map_manager.cpp
                                  src/lines_manager.cpp
                                  src/laserscan_processor.cpp
)

target_link_libraries(db_comparator_node ${catkin_LIBRARIES}
                                         ${PCL_LIBRARIES}
) 

