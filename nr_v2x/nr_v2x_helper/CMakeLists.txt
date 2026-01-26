cmake_minimum_required(VERSION 3.0.2)
project(nr_v2x_helper)

add_compile_options(-O3 -Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES nr_v2x_helper
)

include_directories(${catkin_INCLUDE_DIRS} include)

add_library(nr_v2x_helper
  src/crc16.cpp
)
target_link_libraries(nr_v2x_helper ${catkin_LIBRARIES})

install(TARGETS nr_v2x_helper
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
