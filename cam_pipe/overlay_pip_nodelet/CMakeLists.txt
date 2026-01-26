cmake_minimum_required(VERSION 3.0.2)
project(overlay_pip_nodelet)

add_compile_options(-O3 -Wall -Wextra -Wno-unused-parameter -Wno-unknown-pragmas)
add_definitions(-D__STRICT_ANSI__)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  pluginlib
  image_transport
  cv_bridge
  sensor_msgs
  std_msgs
  tod_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES overlay_pip_core overlay_pip_nodelet
  CATKIN_DEPENDS roscpp nodelet pluginlib image_transport cv_bridge sensor_msgs std_msgs tod_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Core (공통 로직)
add_library(overlay_pip_core
  src/overlay_pip_core.cpp
)
target_link_libraries(overlay_pip_core
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

## Nodelet 래퍼
add_library(overlay_pip_nodelet
  src/overlay_pip_nodelet.cpp
)
target_link_libraries(overlay_pip_nodelet
  overlay_pip_core
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

## Standalone node
add_executable(overlay_pip_node src/overlay_pip_node.cpp)
target_link_libraries(overlay_pip_node
  overlay_pip_core
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

install(TARGETS overlay_pip_core overlay_pip_nodelet overlay_pip_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
