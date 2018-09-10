cmake_minimum_required(VERSION 2.8.3)
project(workspace_analyze)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
)

catkin_package(
  CATKIN_DEPENDS
	roscpp 
	std_msgs
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(workspace_analyze src/workspace_analyze.cpp)
target_link_libraries(workspace_analyze ${catkin_LIBRARIES})
