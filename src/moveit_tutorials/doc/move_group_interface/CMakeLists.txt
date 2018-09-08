add_executable(move_group_interface_tutorial src/move_group_interface_tutorial.cpp)
target_link_libraries(move_group_interface_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_interface_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


add_executable(move_group_interface_ur3 src/move_group_interface_ur3.cpp)
target_link_libraries(move_group_interface_ur3 ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_interface_ur3 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(move_group_interface_ur5 src/move_group_interface_ur5.cpp)
target_link_libraries(move_group_interface_ur5 ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_interface_ur5 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
