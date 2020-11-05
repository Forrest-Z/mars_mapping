cmake_minimum_required(VERSION 3.0.2)
project(mars_slam)
# set(CMAKE_CXX_STANDARD 14) 
add_compile_options(-std=c++14 -O3)

list( APPEND CMAKE_MODULE_PATH /home/ponpon/g2o/cmake_modules ) 

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  geometry_msgs
  roscpp
  sensor_msgs
  tf
  tf_conversions
  eigen_conversions
  mars_srvs
)

SET( G2O_ROOT /usr/local/include/g2o )
find_package( G2O REQUIRED )
find_package( CSparse REQUIRED )
find_package( Cholmod REQUIRED )

find_package(PCL 1.7 REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mars_local_localization
#  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs tf tf_conversions
#  DEPENDS system_lib
)

link_directories(${G2O_LIBRARY_DIRS})
link_libraries(${G2O_LIBRARIES})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS}
  ${CSPARSE_INCLUDE_DIR}
  ${Cholmod_INCLUDE_DIR}
)
 
add_executable(mars_slam_node src/main.cpp 
                              src/mars_slam_node.cpp 
                              src/pose_extrapolator.cpp
                              src/laserscan_processor.cpp
                              src/map/map_manager.cpp
                              src/map/occ_map.cpp
                              src/map/map_info_viewer.cpp
                              src/motion_filter.cpp
                              src/scan_matching/scan_matcher.cpp
                              src/scan_matching/icp_scan_matcher.cpp 
                              src/loop_closure_detection/loop_closure_detection.cpp
                              src/loop_closure_detection/map_stacks.cpp
                              src/optimization/pose_optimizer.cpp
                              src/optimization/simple_graph.cpp
)

target_link_libraries(mars_slam_node  ${catkin_LIBRARIES}
                                      ${PCL_LIBRARIES}
                                      g2o_core 
                                      g2o_stuff 
                                      g2o_types_slam2d 
                                      g2o_types_slam3d 
                                      g2o_types_sba    
                                      g2o_solver_csparse 
                                      g2o_csparse_extension   
                                      g2o_solver_cholmod
                                      cholmod
                                      ${CSPARSE_LIBRARY}
) 