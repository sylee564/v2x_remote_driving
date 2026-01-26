#include <ros/ros.h>
#include "overlay_pip_nodelet/overlay_pip_core.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "overlay_pip_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  overlay_pip_nodelet::OverlayPipCore core;
  core.init(nh, pnh);

  ROS_INFO("[overlay_pip_node] started");
  ros::spin();
  return 0;
}
