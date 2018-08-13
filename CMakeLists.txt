cmake_minimum_required(VERSION 2.8.3)
project(multirobot_stage)
set(CMAKE_CXX_FLAGS "-std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gmapping
  move_base
  roscpp
  rospy
  rviz
  stage_ros
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf
  tf2_ros
  tf2_geometry_msgs
  eigen_conversions
  costmap_2d
  actionlib
  message_generation
  custom_messages
)
find_package(Eigen3 REQUIRED)

# Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY
  FILES
  RobotPosition.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES multirobot_stage
  CATKIN_DEPENDS gmapping move_base roscpp rospy rviz stage_ros
                  std_msgs geometry_msgs sensor_msgs nav_msgs
                  tf tf2_ros tf2_geometry_msgs eigen_conversions costmap_2d
                  simple_navigation_goals multirobot_map_merge nav_bundle
                  stagebot_2dnav
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS} src
)

add_executable(bouncer src/bouncer.cpp)
target_link_libraries(bouncer ${catkin_LIBRARIES})
add_dependencies(bouncer ${catkin_EXPORTED_TARGETS})

add_executable(conversion_tester src/conversion_tester.cpp)
target_link_libraries(conversion_tester
                      ${catkin_LIBRARIES}
                      ${Eigen3_LIBRARIES})

add_executable(action_node src/action_node.cpp)
target_link_libraries(action_node
                      ${catkin_LIBRARIES}
                      ${Eigen3_LIBRARIES})

add_executable(robot_map src/robot_costmap.cpp)
target_link_libraries(robot_map ${catkin_LIBRARIES})
add_dependencies(robot_map ${catkin_EXPORTED_TARGETS})

add_executable(learning_episode_timer src/learning_episode_timer.cpp)
target_link_libraries(learning_episode_timer ${catkin_LIBRARIES})
add_dependencies(learning_episode_timer ${catkin_EXPORTED_TARGETS})

add_executable(comms_node src/comms_node.cpp)
target_link_libraries(comms_node ${catkin_LIBRARIES})
add_dependencies(comms_node ${catkin_EXPORTED_TARGETS})

add_executable(multiRobotNE src/multiRobotNE.cpp)
target_link_libraries(multiRobotNE ${catkin_LIBRARIES})
add_dependencies(multiRobotNE ${catkin_EXPORTED_TARGETS})

add_executable(rewards_node src/exploration_rewards.cpp)
target_link_libraries(rewards_node ${catkin_LIBRARIES})
add_dependencies(rewards_node ${catkin_EXPORTED_TARGETS})

add_executable(global_odom src/global_odom.cpp)
target_link_libraries(global_odom ${catkin_LIBRARIES})
add_dependencies(global_odom ${catkin_EXPORTED_TARGETS})

add_executable(agent_policy src/agent_policy.cpp)
target_link_libraries(agent_policy ${catkin_LIBRARIES})
add_dependencies(agent_policy ${catkin_EXPORTED_TARGETS})

add_executable(comms_map src/comms_map.cpp)
target_link_libraries(comms_map ${catkin_LIBRARIES})
add_dependencies(comms_map ${catkin_EXPORTED_TARGETS})

add_executable(merged_map_republisher src/merged_map_republisher.cpp)
target_link_libraries(merged_map_republisher ${catkin_LIBRARIES})
add_dependencies(merged_map_republisher ${catkin_EXPORTED_TARGETS})
