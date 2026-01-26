cmake_minimum_required(VERSION 3.0.2)
project(kona_can_bridge)

add_compile_options(-std=c++17 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  can_msgs
  std_msgs
  message_runtime
  kona_driver_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp can_msgs std_msgs kona_driver_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(kona_can_node
  src/kona_can_node.cpp
  src/main.cpp
)

target_link_libraries(kona_can_node
  ${catkin_LIBRARIES}
)

install(TARGETS kona_can_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
