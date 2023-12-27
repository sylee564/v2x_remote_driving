cmake_minimum_required(VERSION 3.0.2)
project(tod_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  tod_msgs
  tod_core
  tod_network
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs   std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tod_control
 CATKIN_DEPENDS message_generation message_runtime roscpp std_msgs tod_msgs tod_network
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############
option(OPERATOR "Build operator targets" ON)
if (${OPERATOR})
    add_subdirectory(src/operator)
else()
    message("skipping ${PROJECT_NAME} operator targets")
endif()

option(VEHICLE "Build vehicle targets" ON)
if (${VEHICLE})
    add_subdirectory(src/vehicle)
else()
    message("skipping ${PROJECT_NAME} vehicle targets")
endif()


## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY include/${PROJECT_NAME}/
  launch
  # myfile2
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
