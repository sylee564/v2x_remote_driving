#pragma once
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "h26x_decoder/h26x_decoder.hpp"

namespace h26xdec {

class H26xDecoderNodelet final : public nodelet::Nodelet {
public:
  H26xDecoderNodelet() = default;
  ~H26xDecoderNodelet() override = default;

private:
  void onInit() override {
    ros::NodeHandle nh  = getNodeHandle();
    ros::NodeHandle pnh = getPrivateNodeHandle();
    try {
      node_.reset(new H26xDecoderNode(nh, pnh));
      NODELET_INFO("H26xDecoderNodelet initialized");
    } catch (const std::exception& e) {
      NODELET_FATAL("Failed to init H26xDecoderNodelet: %s", e.what());
      throw;
    }
  }
  std::unique_ptr<H26xDecoderNode> node_;
};

} // namespace h26xdec

PLUGINLIB_EXPORT_CLASS(h26xdec::H26xDecoderNodelet, nodelet::Nodelet)
