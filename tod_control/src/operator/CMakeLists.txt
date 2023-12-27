cmake_minimum_required(VERSION 3.0.2)

set(TARGETS ${TARGETS} OperatorCommandCreator)
add_executable(OperatorCommandCreator CommandCreatorNode.cpp CommandCreator.cpp)
target_compile_features(OperatorCommandCreator PRIVATE cxx_std_17)
add_dependencies(OperatorCommandCreator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(OperatorCommandCreator ${catkin_LIBRARIES})
target_include_directories(OperatorCommandCreator PRIVATE ${catkin_INCLUDE_DIRS})


set(TARGETS ${TARGETS} ForceFeedbackController)
add_executable(ForceFeedbackController ForceFeedbackControllerNode.cpp ForceFeedbackController.cpp)
add_dependencies(ForceFeedbackController ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ForceFeedbackController ${catkin_LIBRARIES})
target_include_directories(ForceFeedbackController PRIVATE ${catkin_INCLUDE_DIRS})

set(TARGETS ${TARGETS} OperatorControlCommandSender)
add_executable(OperatorControlCommandSender ControlCommandSender.cpp)
add_dependencies(OperatorControlCommandSender ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(OperatorControlCommandSender ${catkin_LIBRARIES})
target_include_directories(OperatorControlCommandSender PRIVATE ${catkin_INCLUDE_DIRS})

install(TARGETS ${TARGETS}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)