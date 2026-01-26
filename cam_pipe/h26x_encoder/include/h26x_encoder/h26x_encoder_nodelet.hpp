#pragma once
#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "h26x_encoder/h26x_encoder.hpp"

namespace encoder {

class H26xEncoderNodelet final : public nodelet::Nodelet {
public:
  H26xEncoderNodelet() = default;
  ~H26xEncoderNodelet() override = default;

private:
  void onInit() override {
    ros::NodeHandle nh  = getNodeHandle();
    ros::NodeHandle pnh = getPrivateNodeHandle();
    try {
      node_.reset(new H26xEncoderNode(nh, pnh));
      NODELET_INFO("H26xEncoderNodelet initialized");
    } catch (const std::exception& e) {
      NODELET_FATAL("Failed to init H26xEncoderNodelet: %s", e.what());
      throw;
    }
  }

  std::unique_ptr<H26xEncoderNode> node_;
};

} // namespace encoder

PLUGINLIB_EXPORT_CLASS(encoder::H26xEncoderNodelet, nodelet::Nodelet)
