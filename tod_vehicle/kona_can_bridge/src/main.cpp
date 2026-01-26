#include <ros/ros.h>
#include "kona_can_bridge/kona_can_node.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "kona_can_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  kona_can::KonaCanNode node(nh, pnh);
  ros::spin();
  return 0;
}
