cmake_minimum_required(VERSION 3.0.2)
project(franka_trajectory_exec)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_ros_planning_interface
  trajectory_msgs
  std_msgs
  moveit_visual_tools
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS
    roscpp
    moveit_ros_planning_interface
    trajectory_msgs
    std_msgs
    moveit_visual_tools
)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(circle_trajectory_node src/circle_trajectory.cpp)


## Add dependencies if you generate messages or services
add_dependencies(circle_trajectory_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(circle_trajectory_node
  ${catkin_LIBRARIES}
)