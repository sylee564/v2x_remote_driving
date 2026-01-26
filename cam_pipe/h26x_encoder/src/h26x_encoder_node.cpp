#include "h26x_encoder/h26x_encoder.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "h26x_encoder_node");
  ros::NodeHandle nh, pnh("~");
  try {
    encoder::H26xEncoderNode node(nh, pnh);
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL("h26x_encoder_node fatal: %s", e.what());
    return 1;
  }
  return 0;
}
