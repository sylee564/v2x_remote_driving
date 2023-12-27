set(TARGETS ${TARGETS} VehicleDataReceiver)
add_executable(VehicleDataReceiver VehicleDataReceiver.cpp)
target_link_libraries(VehicleDataReceiver ${catkin_LIBRARIES})
target_include_directories(VehicleDataReceiver PRIVATE ${catkin_INCLUDE_DIRS})
set_property(TARGET VehicleDataReceiver PROPERTY CXX_STANDARD 17)

set(TARGETS ${TARGETS} PVDReceiver)
add_executable(PVDReceiver PVDReceiver.cpp)
target_link_libraries(PVDReceiver ${catkin_LIBRARIES})
target_include_directories(PVDReceiver PRIVATE ${catkin_INCLUDE_DIRS})
set_property(TARGET PVDReceiver PROPERTY CXX_STANDARD 17)

install(TARGETS ${TARGETS}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
