add_executable(planning_scene_tutorial src/planning_scene_tutorial.cpp)
target_link_libraries(planning_scene_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS planning_scene_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(planning_scene_ur3 src/planning_scene_ur3.cpp)
target_link_libraries(planning_scene_ur3 ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS planning_scene_ur3 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
