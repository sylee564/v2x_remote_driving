#include <ros/ros.h>
#include "gscam_cuda_pip/gscam_cuda_pip_node.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gscam_cuda_pip_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try {
    gscam_cuda_pip::GscamCudaPipNode node(nh, pnh);
    node.run();  // configure -> initStream -> publishLoop -> cleanup
  } catch (const std::exception& e) {
    ROS_FATAL_STREAM("[gscam_cuda_pip] Unhandled exception: " << e.what());
    return 1;
  } catch (...) {
    ROS_FATAL("[gscam_cuda_pip] Unknown exception");
    return 1;
  }

  return 0;
}
