cmake_minimum_required(VERSION 3.0.2)
project(ros_image_calibrator)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
  image_transport
  camera_info_manager
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  scripts/image_calibrator.py
  scripts/verify_calibration_from_topic.py
  scripts/avm_homography_from_topic.py
  scripts/yaml_utils.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
