set(QT_MINIMUM_VERSION 5.9)
find_package(Qt5 REQUIRED COMPONENTS QuickWidgets)
if(Qt5_VERSION VERSION_LESS QT_MINIMUM_VERSION)
    MESSAGE(FATAL_ERROR "Minimum supported Qt version: ${QT_MINIMUM_VERSION}.
        Installed version: ${Qt5_VERSION}.")
endif()

set(VIRTUAL_SOURCES
    VirtualInputDevice.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../General/InputDevice.cpp)

qt5_wrap_ui(VIRTUAL_HEADERS_MOC ui/joystickwindow.ui)
qt5_add_resources(VIRTUAL_RESOURCES resources/qml.qrc)
qt5_wrap_cpp(VIRTUAL_SOURCES VirtualInputDevice.h)

set(TARGET VirtualInputDevice)
add_library(VirtualInputDevice
    ${VIRTUAL_SOURCES}
    ${VIRTUAL_HEADERS_MOC}
    ${VIRTUAL_RESOURCES}
)  

target_include_directories(VirtualInputDevice
    PRIVATE
        resources
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../General
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(VirtualInputDevice
    PRIVATE
        Qt5::QuickWidgets
)

install(TARGETS VirtualInputDevice
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

set(DEVICE_TARGETS ${DEVICE_TARGETS} VirtualInputDevice PARENT_SCOPE)
