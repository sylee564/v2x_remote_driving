find_package(Qt5 REQUIRED COMPONENTS Widgets Core Network Test)
set(QT_MINIMUM_VERSION 5.7)
if(Qt5_VERSION VERSION_LESS QT_MINIMUM_VERSION)
    MESSAGE(FATAL_ERROR "Minimum supported Qt version: ${QT_MINIMUM_VERSION}.
        Installed version: ${Qt5_VERSION}.")
endif()

find_package(yaml-cpp REQUIRED)

set(TARGET_OPERATOR_NAME OperatorManager)

set(OPERATOR_MANAGER_WINDOW_UI ../common/ui/v2xManagerWindow.ui)
set(OPERATOR_H OperatorManager.h ../common/v2xManagerWindow.h)
set(OPERATOR_CPP
    OperatorManagerNode.cpp
    OperatorManager.cpp
    ../common/IPv4ValidityChecker.cpp
    ../common/v2xManagerWindow.cpp
    ../common/v2xHeaderDeserializer.h
    ../common/QtWidgetGroup.cpp)

qt5_wrap_cpp(OPERATOR_CPP ${OPERATOR_H})
qt5_wrap_ui(PROJECT_HEADERS_MOC ${OPERATOR_MANAGER_WINDOW_UI})
qt5_add_resources(PROJECT_RESOURCES ../common/ui/resources/resources.qrc)


add_executable(${TARGET_OPERATOR_NAME}
    ${PROJECT_HEADERS_MOC}
    ${OPERATOR_CPP}
    ${PROJECT_RESOURCES})
add_dependencies(${TARGET_OPERATOR_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(${TARGET_OPERATOR_NAME} PRIVATE
    ../common/ui/resources
    ${catkin_INCLUDE_DIRS}  
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ../common/)
target_compile_features(${TARGET_OPERATOR_NAME} PUBLIC cxx_std_17)
target_link_libraries(${TARGET_OPERATOR_NAME}
    PRIVATE
    Qt5::Widgets
    Qt5::Core
    Qt5::Network
    ${catkin_LIBRARIES}
    PahoMqttCpp::paho-mqttpp3
    ${YAML_CPP_LIBRARIES}
    )

install(TARGETS ${TARGET_OPERATOR_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

set(DEVICE_TARGETS ${DEVICE_TARGETS} ${TARGET_OPERATOR_NAME} PARENT_SCOPE)