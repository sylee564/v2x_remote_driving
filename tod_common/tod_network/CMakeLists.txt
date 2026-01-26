cmake_minimum_required(VERSION 3.0.2)
project(tod_network)

# -------------------------
# Build setup
# -------------------------
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# -------------------------
# Catkin & ROS deps
# -------------------------
find_package(catkin REQUIRED COMPONENTS
  roscpp
  # tod_msgs  # 필요시 주석 해제
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp # tod_msgs
)

# ----- System deps -----
find_package(Threads REQUIRED)
find_package(OpenSSL REQUIRED)

# ----- Paho (find_library로 직접 탐색) -----
# 공통 힌트 경로 (x86_64/ARM 공용 후보)
set(_PAHO_HINTS
  /usr/local/lib
  /usr/local/lib/aarch64-linux-gnu
  /usr/lib
  /usr/lib/x86_64-linux-gnu
  /usr/lib/aarch64-linux-gnu
)

# C++ (우선 pp3, 없다면 pp)
find_library(PAHO_CPP_LIB
  NAMES paho-mqttpp3 paho-mqttpp
  HINTS ${_PAHO_HINTS}
)
# C (TLS 쓰면 3as, 아니면 3a)
find_library(PAHO_C_LIB
  NAMES paho-mqtt3as paho-mqtt3a
  HINTS ${_PAHO_HINTS}
)

if (NOT PAHO_CPP_LIB)
  message(FATAL_ERROR "Paho C++ library not found (paho-mqttpp3/paho-mqttpp). \
  Make sure you installed paho.mqtt.cpp to /usr/local/lib.")
endif()

if (NOT PAHO_C_LIB)
  message(FATAL_ERROR "Paho C library not found (paho-mqtt3as/paho-mqtt3a). \
  Make sure you installed paho.mqtt.c to /usr/local/lib.")
endif()

# ----- include dirs -----
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/local/include
)

# ----- target -----
add_library(${PROJECT_NAME}
  src/mqtt_client.cpp
  src/mqtt_client_templated.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# ----- link -----
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PAHO_CPP_LIB}
  ${PAHO_C_LIB}
  OpenSSL::SSL
  OpenSSL::Crypto
  Threads::Threads
)

# -------------------------
# Install
# -------------------------
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
