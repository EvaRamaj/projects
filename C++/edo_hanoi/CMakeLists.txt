# copied from edo_pnp/CMakeLists.txt
# all outcommented lines removed ~Jose

cmake_minimum_required(VERSION 2.8.3)
project(edo_hanoi)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  robot_state_publisher
  edo_utils
  edo_pnp
  std_msgs
  message_generation
  tf
)

add_service_files(
  FILES
  HanoiState.srv
)

#add_message_files(
#  FILES
#)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/edo_hanoi_node.cpp src/hanoi.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})





