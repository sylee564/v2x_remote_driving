#include <ros/ros.h>
#include "tod_vehicle_bridge/kona_control_probe_node.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "kona_control_probe_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tod_kona_bridge::KonaControlProbeNode node(nh, pnh);
  ros::spin();
  return 0;
}
