cmake_minimum_required(VERSION 3.0.2)
project(nr_v2x_interface LANGUAGES CXX)

# ---- C++ / Build type ----
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O3)
endif()

# 공유 라이브러리/플러그인 호환
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ---- Dependencies ----
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tcpip_msgs
  tod_msgs
  nr_v2x_helper
  nr_v2x_msgs
  diagnostic_msgs
  h26x_encoder
)

# Boost.Asio가 system 라이브러리를 필요로 할 수 있음
find_package(Boost REQUIRED COMPONENTS system)

# 일부 환경에서 pthread 필요
find_package(Threads REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES nr_v2x_interface
  CATKIN_DEPENDS roscpp tcpip_msgs tod_msgs nr_v2x_helper nr_v2x_msgs diagnostic_msgs h26x_encoder
)

# ---- Library ----
add_library(nr_v2x_interface
  src/socket.cpp
  src/endpoint.cpp
  src/tcp_client.cpp
  src/tcp_server.cpp
  src/tcp_socket.cpp
  src/udp_socket.cpp
  src/video_sender.cpp
)

target_include_directories(nr_v2x_interface
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${catkin_INCLUDE_DIRS}
)

# 메시지/서비스 생성 이후에 빌드되도록
add_dependencies(nr_v2x_interface ${catkin_EXPORTED_TARGETS})

target_link_libraries(nr_v2x_interface
  PUBLIC
    ${catkin_LIBRARIES}
    Boost::system
    Threads::Threads
)

# ---- Executable ----
add_executable(v2x_interface_node
  src/main.cpp
  src/driver_tcpip.cpp
)

add_dependencies(v2x_interface_node nr_v2x_interface ${catkin_EXPORTED_TARGETS})

target_link_libraries(v2x_interface_node
  PRIVATE
    nr_v2x_interface
    ${catkin_LIBRARIES}
    Boost::system
    Threads::Threads
)

# ---- Install ----
install(TARGETS
  nr_v2x_interface
  v2x_interface_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# 런치 파일도 설치(있다면)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch")
endif()
