#include "h26x_decoder/h26x_decoder.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "h26x_decoder_node");
  ros::NodeHandle nh, pnh("~");
  h26xdec::H26xDecoderNode node(nh, pnh);
  ros::spin();
  return 0;
}
