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
  geometry_msgs
  sensor_msgs
  tf
  costmap_2d
)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(bouncer src/bouncer.cpp)
target_link_libraries(bouncer ${catkin_LIBRARIES})
add_dependencies(bouncer ${catkin_EXPORTED_TARGETS})

add_executable(robot_map src/robot_costmap.cpp)
target_link_libraries(robot_map ${catkin_LIBRARIES})
add_dependencies(robot_map ${catkin_EXPORTED_TARGETS})
