#include <ros/ros.h>
#include "ControlCommandSender.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ControlCommandSender");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  tod_control::ControlCommandSender node(nh, pnh);

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
