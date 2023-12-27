include_directories(${catkin_INCLUDE_DIRS})

add_executable(kona_control_send kona_control_send_node.cpp)
add_dependencies(kona_control_send ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kona_control_send ${catkin_LIBRARIES})

add_executable(kona_data_recv kona_data_recv_node.cpp)
add_dependencies(kona_data_recv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kona_data_recv ${catkin_LIBRARIES})

install(TARGETS
    kona_control_send
    kona_data_recv
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
