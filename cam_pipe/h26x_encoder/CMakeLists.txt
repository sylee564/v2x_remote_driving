cmake_minimum_required(VERSION 3.0.2)
project(h26x_encoder)

add_compile_options(-std=c++14 -O2 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp sensor_msgs std_msgs cv_bridge image_transport message_generation nodelet
)

add_message_files(FILES
  EncodedFrame.msg
  EncoderStats.msg
)

generate_messages(DEPENDENCIES std_msgs)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs message_runtime nodelet
  INCLUDE_DIRS include
  LIBRARIES h26x_encoder_core h26x_encoder_nodelet
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
)
link_directories(${GST_LIBRARY_DIRS})

# 1) 코어 라이브러리 (기존 구현 파일 포함)
add_library(h26x_encoder_core
  src/h26x_encoder.cpp
)
add_dependencies(h26x_encoder_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(h26x_encoder_core ${catkin_LIBRARIES} ${GST_LIBRARIES})

# 2) 실행형 노드
add_executable(h26x_encoder_node src/h26x_encoder_node.cpp)
add_dependencies(h26x_encoder_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(h26x_encoder_node h26x_encoder_core ${catkin_LIBRARIES} ${GST_LIBRARIES})

# 3) 노드렛 라이브러리
add_library(h26x_encoder_nodelet
  src/h26x_encoder_nodelet.cpp
)
add_dependencies(h26x_encoder_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(h26x_encoder_nodelet h26x_encoder_core ${catkin_LIBRARIES} ${GST_LIBRARIES})

# install
install(TARGETS h26x_encoder_core h26x_encoder_node h26x_encoder_nodelet
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
