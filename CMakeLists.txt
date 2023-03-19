add_executable(gluon_control_moveit src/gluon_control_moveit.cpp)
target_link_libraries(gluon_control_moveit ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS gluon_control_moveit DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
