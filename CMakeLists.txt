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
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES multirobot_stage
  CATKIN_DEPENDS gmapping move_base roscpp rospy rviz stage_ros
                  std_msgs geometry_msgs sensor_msgs nav_msgs
                  tf tf2_ros tf2_geometry_msgs eigen_conversions costmap_2d
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

add_executable(multiRobotNE src/multiRobotNE.cpp)
target_link_libraries(multiRobotNE ${catkin_LIBRARIES})
add_dependencies(multiRobotNE ${catkin_EXPORTED_TARGETS})
