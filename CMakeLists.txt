# gluon_control_moveit.cpp
add_executable(gluon_control_moveit src/gluon_control_moveit.cpp)
target_link_libraries(gluon_control_moveit ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS gluon_control_moveit DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# robot_model_and_robot_state_moveit.cpp
add_executable(robot_model_and_robot_state_moveit src/robot_model_and_robot_state_moveit.cpp)
target_link_libraries(robot_model_and_robot_state_moveit ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS robot_model_and_robot_state_moveit DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# gluon_final.cpp
add_executable(gluon_final src/gluon_final.cpp)
target_link_libraries(gluon_final ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS gluon_final DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
