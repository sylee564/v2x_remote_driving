# 소스
set(VEHICLE_NODE ControlCmdReceiverNode.cpp)
set(VEHICLE_CPP
  ControlCmdReceiver.cpp
  ../common/IPv4ValidityChecker.cpp
)
set(TARGET_VEHICLE_NAME ControlCmdReceiver)

add_executable(${TARGET_VEHICLE_NAME}
  ${VEHICLE_NODE}
  ${VEHICLE_CPP}
)

add_dependencies(${TARGET_VEHICLE_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_include_directories(${TARGET_VEHICLE_NAME} PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ../common
  /usr/local/include     # paho 헤더 기본 설치 경로
)

target_compile_features(${TARGET_VEHICLE_NAME} PUBLIC cxx_std_17)

# --- 링크: find_library 로 찾은 실제 .so ---
target_link_libraries(${TARGET_VEHICLE_NAME}
  PRIVATE
  ${catkin_LIBRARIES}
  ${PAHO_CPP_LIB}     # e.g. /usr/local/lib/libpaho-mqttpp3.so
  ${PAHO_C_LIB}       # e.g. /usr/local/lib/libpaho-mqtt3as.so (or 3a)
  Threads::Threads
)

install(TARGETS ${TARGET_VEHICLE_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
