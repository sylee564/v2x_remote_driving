cmake_minimum_required(VERSION 3.0.2)
set(PROJECT_NAME tod_core)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs)
find_package(yaml-cpp REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME} ${YAML_CPP_LIBRARIES}
    CATKIN_DEPENDS roscpp
    DEPENDS YAML_CPP
    )

add_library(${PROJECT_NAME}
    src/BaseParameterHandler.cpp
)
target_include_directories(${PROJECT_NAME} 
    PRIVATE
        ${catkin_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIRS}
        src
    PUBLIC 
        include
)

target_compile_definitions(${PROJECT_NAME} PUBLIC cxx_std_17)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
    PRIVATE
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    PUBLIC
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
