cmake_minimum_required(VERSION 2.8.3)
project(slam_project)

## Find catkin and any catkin packages
find_package(catkin REQUIRED 
  roscpp 
  std_msgs 
  message_generation
  roslib
  genmsg
)

find_package(Eigen3 REQUIRED)

## Declare ROS messages and services
add_message_files(
  FILES
  Robot_Path_Map.msg
  Robot_Odometry.msg
  Robot_Measure.msg
)
add_service_files(
  FILES
  requestBarcode.srv
)

## Generate added messages and services
generate_messages(
  DEPENDENCIES 
  std_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime roslib
)

# re-define complier and linker flags. o/w an error due to "gcov" (GCC test coverage program) would happen
SET(GCC_COVERAGE_COMPILE_FLAGS
  "-fprofile-arcs -ftest-coverage"
)

SET(GCC_COVERAGE_LINK_FLAGS
  "-lgcov -fprofile-arcs"
)

SET(CMAKE_CXX_FLAGS  
  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS} -std=c++14" 
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
  src/fast_slam.cpp
  src/fast_slam2.cpp
  src/multi_fast_slam.cpp
  src/multi_fast_slam2.cpp
  src/dec_multi_fast_slam.cpp
  src/motion_models/velocity_motion_model.cpp
  src/measurement_models/feature_measurement_model.cpp
  src/robot_models/interface_robot_model.cpp
  src/robot_models/mobile_robot_2dmodel.cpp
  src/map_models/feature_map_2dmodel.cpp
)

add_executable(points_and_lines src/points_and_lines.cpp)
target_link_libraries(points_and_lines ${catkin_LIBRARIES})

target_link_libraries(slam_runner_node
  ${catkin_LIBRARIES}
)

add_executable(basic_shapes src/basic_shapes.cpp)
target_link_libraries(basic_shapes ${catkin_LIBRARIES})

add_dependencies(points_and_lines slam_project_generate_messages_cpp)
add_dependencies(data_reader_node slam_project_generate_messages_cpp)
add_dependencies(slam_runner_node slam_project_generate_messages_cpp)