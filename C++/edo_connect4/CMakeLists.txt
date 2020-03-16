# copied from edo_pnp/CMakeLists.txt
# all outcommented lines removed ~Jose

cmake_minimum_required(VERSION 2.8.3)
project(edo_connect4)

## Compile as C++14
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  robot_state_publisher
  edo_gazebo
  edo_gripper
  edo_gripper_moveit
  edo_utils
  hector_sensors_description
  edo_games_sim
  edo_pnp
  openni2_camera
  std_msgs
  message_generation
  tf
  sensor_msgs
)

add_service_files(
  FILES
  Connect4State.srv
  ModelUpdate.srv
  Reset.srv
  Dice.srv
  EdoSpeak.srv
  PlayerDetector.srv
  DialogFlow.srv
)

add_message_files(
  FILES
  DialogflowResult.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

###TEST
## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/dialogflow_client.py
  scripts/mic_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
foreach (dir launch config)
    install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
###ENDTEST

add_executable(${PROJECT_NAME}_node src/edo_connect4_node.cpp src/connect4.cpp src/connect4_graph.cpp src/connect4_monte_carlo.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

#add_executable(storing_poses src/joint_movements.h)
#target_link_libraries(storing_poses ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_test test/connect4_test.cpp src/connect4.cpp src/connect4_graph.cpp src/connect4_monte_carlo.cpp)
target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_interactive src/edo_connect4_interactive.cpp src/connect4.cpp src/connect4_graph.cpp src/connect4_monte_carlo.cpp include/jokes_db.h)
target_link_libraries(${PROJECT_NAME}_interactive ${catkin_LIBRARIES})
