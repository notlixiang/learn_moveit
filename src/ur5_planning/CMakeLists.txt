cmake_minimum_required(VERSION 2.8.3)
project(ur5_planning)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    moveit_visual_tools
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    EIGEN3
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(planning_in_cabin src/planning_in_cabin.cpp)
target_link_libraries(planning_in_cabin ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS planning_in_cabin DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
