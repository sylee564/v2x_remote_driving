#pragma once
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include "overlay_pip_nodelet/overlay_pip_core.hpp"

namespace overlay_pip_nodelet {

class OverlayPipNodelet : public nodelet::Nodelet {
public:
  OverlayPipNodelet() = default;
  ~OverlayPipNodelet() override = default;

  void onInit() override {
    ros::NodeHandle& nh  = getNodeHandle();
    ros::NodeHandle& pnh = getPrivateNodeHandle();
    core_.init(nh, pnh);
    NODELET_INFO("[overlay_pip_nodelet] onInit done");
  }

private:
  OverlayPipCore core_;
};

} // namespace overlay_pip_nodelet
