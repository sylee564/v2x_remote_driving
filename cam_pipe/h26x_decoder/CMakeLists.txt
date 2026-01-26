cmake_minimum_required(VERSION 3.0.2)
project(h26x_decoder)

add_compile_options(-std=c++14 -O2 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs sensor_msgs image_transport cv_bridge nodelet
  h26x_encoder   # for EncodedFrame.msg
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs image_transport cv_bridge nodelet h26x_encoder
  INCLUDE_DIRS include
  LIBRARIES h26x_decoder_core h26x_decoder_nodelet
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
)
link_directories(${GST_LIBRARY_DIRS})

# core lib
add_library(h26x_decoder_core
  src/h26x_decoder.cpp
)
target_link_libraries(h26x_decoder_core ${catkin_LIBRARIES} ${GST_LIBRARIES})

# node
add_executable(h26x_decoder_node src/h26x_decoder_node.cpp)
target_link_libraries(h26x_decoder_node h26x_decoder_core ${catkin_LIBRARIES} ${GST_LIBRARIES})

# nodelet
add_library(h26x_decoder_nodelet
  src/h26x_decoder_nodelet.cpp
)
target_link_libraries(h26x_decoder_nodelet h26x_decoder_core ${catkin_LIBRARIES} ${GST_LIBRARIES})

# install
install(TARGETS h26x_decoder_core h26x_decoder_node h26x_decoder_nodelet
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
