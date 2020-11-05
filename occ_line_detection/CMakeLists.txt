cmake_minimum_required(VERSION 3.0.2)
project(occ_line_detection)
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-std=c++14 -O3)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  std_msgs
  message_generation
  mars_srvs
  hough_line_msgs
)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

catkin_package(
  # INCLUDE_DIRS include
#  LIBRARIES occ_line_detection
#  CATKIN_DEPENDS nav_msgs roscpp std_msgs
#  DEPENDS system_lib
  # CATKIN_DEPENDS message_runtime 
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(occ_line_detector src/main.cpp 
                                 src/occ_line_detector.cpp
)

target_link_libraries(occ_line_detector ${catkin_LIBRARIES}
) 

add_executable(test_node src/test.cpp 
)

target_link_libraries(test_node ${catkin_LIBRARIES}
) 