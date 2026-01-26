cmake_minimum_required(VERSION 3.0.2)
project(tod_vehicle_bridge)

# C++17
add_compile_options(-std=c++17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Catkin
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf
  tod_msgs
  kona_driver_msgs
  tod_helper
)

find_package(Threads REQUIRED)

## Catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs tf tod_msgs kona_driver_msgs tod_helper
)

## Include dirs
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Sources
# speed_governor.cpp 는 현재 헤더온리여도 OK. 추후 확장 대비해 타겟에 포함.
set(SRC
  src/kona_control_probe_node.cpp
  src/main.cpp
)

## Executable
add_executable(kona_control_probe_node ${SRC})
add_dependencies(kona_control_probe_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kona_control_probe_node
  ${catkin_LIBRARIES}
  Threads::Threads
)

## Install
install(TARGETS kona_control_probe_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# 런치 파일은 share로 설치해야 roslaunch에서 찾기 편함
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

# (선택) 기타 리소스가 있으면 여기에 추가
# install(FILES config/your.yaml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

