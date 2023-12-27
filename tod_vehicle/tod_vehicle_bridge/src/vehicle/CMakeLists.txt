cmake_minimum_required(VERSION 3.0.2)

set(TARGETS ${TARGETS} VehicleDataToPVD)
add_executable(VehicleDataToPVD vehicle_to_PVD_Node.cpp)
add_dependencies(VehicleDataToPVD ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(VehicleDataToPVD ${catkin_LIBRARIES})
target_include_directories(VehicleDataToPVD PRIVATE ${catkin_INCLUDE_DIRS})

set(TARGETS ${TARGETS} VehicleDataSender)
add_executable(VehicleDataSender VehicleDataSender.cpp)
target_link_libraries(VehicleDataSender ${catkin_LIBRARIES})
target_include_directories(VehicleDataSender PRIVATE ${catkin_INCLUDE_DIRS})
set_property(TARGET VehicleDataSender PROPERTY CXX_STANDARD 17)

set(TARGETS ${TARGETS} PVDSender)
add_executable(PVDSender PVDSender.cpp)
target_link_libraries(PVDSender ${catkin_LIBRARIES})
target_include_directories(PVDSender PRIVATE ${catkin_INCLUDE_DIRS})
set_property(TARGET PVDSender PROPERTY CXX_STANDARD 17)

install(TARGETS ${TARGETS}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
