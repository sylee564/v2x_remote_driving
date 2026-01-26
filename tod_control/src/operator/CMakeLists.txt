cmake_minimum_required(VERSION 3.0.2)

set(TARGETS ${TARGETS} OperatorCommandCreator)
add_executable(OperatorCommandCreator command_creator_node.cpp command_creator.cpp)
target_compile_features(OperatorCommandCreator PRIVATE cxx_std_17)
add_dependencies(OperatorCommandCreator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(OperatorCommandCreator ${catkin_LIBRARIES})
target_include_directories(OperatorCommandCreator PRIVATE ${catkin_INCLUDE_DIRS})


set(TARGETS ${TARGETS} ForceFeedbackController)
add_executable(ForceFeedbackController force_feedback_controller_node.cpp force_feedback_controller.cpp)
add_dependencies(ForceFeedbackController ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ForceFeedbackController ${catkin_LIBRARIES})
target_include_directories(ForceFeedbackController PRIVATE ${catkin_INCLUDE_DIRS})

set(TARGETS ${TARGETS} OperatorControlCommandSender)
add_executable(OperatorControlCommandSender ControlCommandSender.cpp ControlCommandSenderNode.cpp)
add_dependencies(OperatorControlCommandSender ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_compile_features(OperatorControlCommandSender PUBLIC cxx_std_17)

target_link_libraries(OperatorControlCommandSender ${catkin_LIBRARIES}   
    ${catkin_LIBRARIES}
    ${PAHO_CPP_LIB}     # e.g. /usr/local/lib/libpaho-mqttpp3.so
    ${PAHO_C_LIB}       # e.g. /usr/local/lib/libpaho-mqtt3as.so (or 3a)
    Threads::Threads
)
target_include_directories(OperatorControlCommandSender PRIVATE ${catkin_INCLUDE_DIRS}
      ../common
  /usr/local/include )

install(TARGETS ${TARGETS}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)