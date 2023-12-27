set(VEHICLE_NODE VehicleManagerXavierNode.cpp)
set(VEHICLE_CPP VehicleManagerXavier.cpp
                ../common/IPv4ValidityChecker.cpp)
set(TARGET_VEHICLE_NAME VehicleManagerXavier)

add_executable(${TARGET_VEHICLE_NAME}
    ${VEHICLE_NODE}
    ${VEHICLE_CPP}
)
add_dependencies(${TARGET_VEHICLE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(${TARGET_VEHICLE_NAME} PRIVATE
    ${catkin_INCLUDE_DIRS}  
    ${CMAKE_CURRENT_SOURCE_DIR}
    ../common/)
target_compile_features(${TARGET_VEHICLE_NAME} PUBLIC cxx_std_17)
target_link_libraries(${TARGET_VEHICLE_NAME}
    PRIVATE
    ${catkin_LIBRARIES}
    PahoMqttCpp::paho-mqttpp3
    Threads::Threads)

install(TARGETS ${TARGET_VEHICLE_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)