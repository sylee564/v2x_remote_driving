cmake_minimum_required(VERSION 3.0.2)
project(tod_control)

add_compile_options(-std=c++17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --- Catkin ---
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  tod_msgs
  tod_core
  tod_network
  tod_helper
)

# --- Paho .so 직접 탐색 ---
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
Make sure you installed paho.mqtt.cpp to /usr/local/lib or system lib path.")
endif()

if (NOT PAHO_C_LIB)
  message(FATAL_ERROR "Paho C library not found (paho-mqtt3as/paho-mqtt3a). \
Make sure you installed paho.mqtt.c to /usr/local/lib or system lib path.")
endif()

find_package(Threads REQUIRED)

# 이 패키지는 자체 msg/srv를 만들지 않으므로 generate_messages() 호출/관련 의존 제거
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs tod_msgs tod_network tod_helper
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/local/include        # paho 헤더 기본 설치 경로
)

# ▶ 타깃은 각 하위 디렉토리에서 정의
# add_subdirectory(src/operator)
add_subdirectory(src/vehicle)

# 리소스/런치 설치
install(DIRECTORY include/${PROJECT_NAME}/
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
