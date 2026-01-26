# 소스
set(VEHICLE_NODE ControlCmdReceiverNode.cpp)   # main 파일명 유지
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
  ${PAHO_MQTT_CPP_INCLUDE_DIRS}  # pkg-config 경로(있으면)
)

target_compile_features(${TARGET_VEHICLE_NAME} PUBLIC cxx_std_17)

# --- 링크: CONFIG 타깃 or pkg-config 라이브러리 ---
target_link_libraries(${TARGET_VEHICLE_NAME}
  ${catkin_LIBRARIES}
  ${PAHO_CPP_LIB}
  ${PAHO_C_LIB}
  OpenSSL::SSL
  OpenSSL::Crypto
  Threads::Threads
)

install(TARGETS ${TARGET_VEHICLE_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
