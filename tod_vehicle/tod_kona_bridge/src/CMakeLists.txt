cmake_minimum_required(VERSION 3.0.2)

include_directories( ../include/tod_kona_bridge
                     ${catkin_INCLUDE_DIRS})

add_executable(kona_driver_report_node 
report_node.cc
steering_report_710.cc
Byte.cc
gear_report_720.cc
acc_report_711.cc
brake_report_713.cc
wheel_report_712.cc
)
add_dependencies(kona_driver_report_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kona_driver_report_node ${catkin_LIBRARIES})

add_executable(kona_driver_command_node
command_node.cc
primary_command_157.cc
second_command_156.cc
Byte.cc
)
add_dependencies(kona_driver_command_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kona_driver_command_node ${catkin_LIBRARIES})

install(TARGETS
    kona_driver_report_node
    kona_driver_command_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)