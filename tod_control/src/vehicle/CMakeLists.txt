set(TARGETS ${TARGETS} VehicleControlCommandReceiver)
add_executable(VehicleControlCommandReceiver ControlCommandReceiver.cpp)
add_dependencies(VehicleControlCommandReceiver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(VehicleControlCommandReceiver ${catkin_LIBRARIES})
target_include_directories(VehicleControlCommandReceiver PRIVATE ${catkin_INCLUDE_DIRS})

install(TARGETS ${TARGETS}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
