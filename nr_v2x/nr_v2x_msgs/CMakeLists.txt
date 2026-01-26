cmake_minimum_required(VERSION 3.0.2)
set(PROJECT_NAME nr_v2x_msgs)
project(${PROJECT_NAME})


find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
    ControlUnitStatus.msg
    CommUnitStatus.msg
    ModemRxStatus.msg
    ModemTxStatus.msg
    V2XDataBase.msg
    V2XStat.msg
    V2XDataConfig.msg
)

add_service_files(
  FILES
  Wsr.srv
  V2XConfig.srv)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)
