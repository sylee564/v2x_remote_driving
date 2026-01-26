#include <ros/ros.h>
#include "nr_v2x_interface/driver_tcpip.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "nr_v2x_interface");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  v2x_interface::driver_tcpip_t node(nh, pnh);

  ros::waitForShutdown();
  return 0;
}
