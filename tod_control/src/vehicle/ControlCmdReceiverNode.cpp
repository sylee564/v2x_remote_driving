#include "ros/ros.h"
#include "ControlCmdReceiver.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "control_cmd_receiver");
  ros::NodeHandle nh;        // global namespace
  // ros::NodeHandle pnh("~"); // 필요 시 파라미터용

  ControlCmdReceiver receiver;
  receiver.start(nh);  // Status 구독 + 퍼블리셔 초기화

  ros::spin();
  return 0;
}
