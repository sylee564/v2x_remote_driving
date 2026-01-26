cmake_minimum_required(VERSION 3.0.2)
project(gscam_cuda_pip)

# C++ 표준
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  pluginlib
  image_transport
  camera_info_manager
  sensor_msgs
  cv_bridge
  tod_msgs
)

# GStreamer (core/app/video 전부 필요)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST        REQUIRED gstreamer-1.0>=1.14)
pkg_check_modules(GST_APP    REQUIRED gstreamer-app-1.0>=1.14)
pkg_check_modules(GST_VIDEO  REQUIRED gstreamer-video-1.0>=1.14)

# OpenCV + CUDA 모듈 (필요한 것만 명시)
find_package(OpenCV 4.5 REQUIRED COMPONENTS
  core imgproc calib3d
  cudaimgproc cudawarping cudaarithm
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gscam_cuda_pip_nodelets
  CATKIN_DEPENDS roscpp nodelet pluginlib image_transport camera_info_manager sensor_msgs cv_bridge tod_msgs
  DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
  ${GST_APP_INCLUDE_DIRS}
  ${GST_VIDEO_INCLUDE_DIRS}
)

link_directories(
  ${GST_LIBRARY_DIRS}
  ${GST_APP_LIBRARY_DIRS}
  ${GST_VIDEO_LIBRARY_DIRS}
)

# --- Node executable ---
add_executable(gscam_cuda_pip_node
  src/main.cpp
  src/gscam_cuda_pip_node.cpp
)
target_link_libraries(gscam_cuda_pip_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${GST_LIBRARIES}
  ${GST_APP_LIBRARIES}
  ${GST_VIDEO_LIBRARIES}
)
add_dependencies(gscam_cuda_pip_node ${catkin_EXPORTED_TARGETS})

# --- Nodelet library ---
add_library(gscam_cuda_pip_nodelets
  src/gscam_cuda_pip_node.cpp
  src/gscam_cuda_pip_nodelet.cpp
)
target_link_libraries(gscam_cuda_pip_nodelets
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${GST_LIBRARIES}
  ${GST_APP_LIBRARIES}
  ${GST_VIDEO_LIBRARIES}
)
add_dependencies(gscam_cuda_pip_nodelets ${catkin_EXPORTED_TARGETS})

# Install
install(TARGETS gscam_cuda_pip_node gscam_cuda_pip_nodelets
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  OPTIONAL
)
