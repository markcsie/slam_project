cmake_minimum_required(VERSION 2.8.3)
project(slam_project)

## Find catkin and any catkin packages
find_package(catkin REQUIRED 
  roscpp 
  std_msgs 
  message_generation
)

find_package(Eigen3 REQUIRED)

## Declare ROS messages and services
#add_message_files(FILES)
#add_service_files(FILES)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime
)

# re-define complier and linker flags. o/w an error due to "gcov" (GCC test coverage program) would happen
SET(GCC_COVERAGE_COMPILE_FLAGS
  "-fprofile-arcs -ftest-coverage"
)

SET(GCC_COVERAGE_LINK_FLAGS
  "-lgcov -fprofile-arcs"
)

SET(CMAKE_CXX_FLAGS  
  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS} -std=c++11" 
)

SET(CMAKE_EXE_LINKER_FLAGS
  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}"
)

include_directories(include 
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

link_directories(
   ${catkin_LIBRARY_DIRS}
)

add_executable(data_reader_node 
  src/data_reader.cpp
)

target_link_libraries(data_reader_node
  ${catkin_LIBRARIES}
)

add_executable(slam_runner_node 
  src/slam_runner.cpp
  src/fast_slam2.cpp
)

target_link_libraries(slam_runner_node
  ${catkin_LIBRARIES}
)