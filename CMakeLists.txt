cmake_minimum_required(VERSION 2.8.3)
project(multirobot_stage)

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
  costmap_2d
)
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS} src
)

add_executable(bouncer src/bouncer.cpp)
target_link_libraries(bouncer ${catkin_LIBRARIES})
add_dependencies(bouncer ${catkin_EXPORTED_TARGETS})

add_executable(map2polar src/map2polar.cpp)
target_link_libraries(map2polar
                      ${catkin_LIBRARIES}
                      ${Eigen3_LIBRARIES}
                      m)

add_executable(action_node src/action_node.cpp)
target_link_libraries(action_node
                      ${catkin_LIBRARIES}
                      ${Eigen3_LIBRARIES}
                      m)
add_executable(robot_map src/robot_costmap.cpp)
target_link_libraries(robot_map ${catkin_LIBRARIES})
add_dependencies(robot_map ${catkin_EXPORTED_TARGETS})

add_executable(learning_episode_timer src/learning_episode_timer.cpp)
target_link_libraries(learning_episode_timer ${catkin_LIBRARIES})
add_dependencies(learning_episode_timer ${catkin_EXPORTED_TARGETS})

add_executable(comms_node src/comms_node.cpp)
target_link_libraries(comms_node ${catkin_LIBRARIES})
add_dependencies(comms_node ${catkin_EXPORTED_TARGETS})
