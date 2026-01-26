cmake_minimum_required(VERSION 3.10)
project(tod_manager)

# ===================== C++ / Qt 자동 기능 =====================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Qt 자동 처리
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTORCC ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tod_msgs
  tod_core
  nr_v2x_msgs
  tcpip_msgs       
  roslib
  tod_helper
  kona_driver_msgs
  lanelet2_core
  lanelet2_io
  lanelet2_projection
)

catkin_package(
  CATKIN_DEPENDS roscpp roslib tod_msgs tod_core tcpip_msgs nr_v2x_msgs tod_helper kona_driver_msgs
    lanelet2_core lanelet2_io lanelet2_projection
)

set(QT_MINIMUM_VERSION 5.7)
find_package(Qt5 REQUIRED COMPONENTS
  Widgets
  Core
  Network
  PrintSupport
  WebEngineWidgets
  WebChannel
)
find_package(OpenGL REQUIRED)
find_package(assimp REQUIRED)

## ========== 포함 경로 ==========
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include          # include/tod_manager/*
  ${CMAKE_CURRENT_SOURCE_DIR}/src/common       # 소스에서 직접 include하는 경우 대비
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party
  ${CMAKE_CURRENT_SOURCE_DIR}/resources
  ${CMAKE_CURRENT_BINARY_DIR}                  # ui_*.h / moc_* 포함
)

## ========== Qt UI / MOC / 리소스 ==========
set(UI_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/ui/v2xManagerWindow.ui
)

set(MOC_HEADERS
  ${CMAKE_CURRENT_SOURCE_DIR}/include/tod_manager/Manager.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/tod_manager/v2xManagerWindow.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/tod_manager/map_bridge.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/tod_manager/MapCanvasGL.h 
)

qt5_wrap_ui(UI_GENERATED ${UI_FILES})
qt5_wrap_cpp(MOC_GENERATED ${MOC_HEADERS})
# qrc
set(QRC_FILE ${CMAKE_CURRENT_SOURCE_DIR}/resources/resources.qrc)
qt5_add_resources(QRC_GENERATED ${QRC_FILE})

## ========== 소스 ==========
set(SRC_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ManagerNode.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Manager.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/common/v2xManagerWindow.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/common/IPv4ValidityChecker.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/common/QtWidgetGroup.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/common/MapCanvasGL.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/qcustomplot.cpp
)

## ========== 타겟 ==========
add_executable(tod_manager_node
  ${SRC_FILES}
  ${UI_GENERATED}
  ${MOC_GENERATED}
  ${QRC_GENERATED}
)

add_dependencies(tod_manager_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(tod_manager_node
  ${catkin_LIBRARIES}
  Qt5::Widgets
  Qt5::Core
  Qt5::Network
  Qt5::PrintSupport
  Qt5::WebEngineWidgets
  Qt5::WebChannel
  OpenGL::GL
  assimp
)

# 실행 파일 설치
install(TARGETS tod_manager_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/daconfig
)
