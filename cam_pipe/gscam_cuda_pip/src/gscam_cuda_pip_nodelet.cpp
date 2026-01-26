#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <thread>
#include <memory>

#include "gscam_cuda_pip/gscam_cuda_pip_node.hpp"

namespace gscam_cuda_pip {

class GscamCudaPipNodelet final : public nodelet::Nodelet {
public:
  GscamCudaPipNodelet() = default;
  ~GscamCudaPipNodelet() override {
    // 안전한 종료 순서: 스트림 정리 → 스레드 조인
    try {
      if (node_) node_->cleanupStream();
    } catch (const std::exception& e) {
      ROS_WARN_STREAM("[gscam_cuda_pip][nodelet] cleanupStream exception: " << e.what());
    } catch (...) {
      ROS_WARN("[gscam_cuda_pip][nodelet] cleanupStream unknown exception");
    }

    if (worker_.joinable()) {
      worker_.join();
    }
  }

  void onInit() override {
    ros::NodeHandle nh      = getNodeHandle();
    ros::NodeHandle pnh     = getPrivateNodeHandle();

    try {
      node_ = std::make_unique<GscamCudaPipNode>(nh, pnh);

      // configure: 파라미터/환경변수 로드
      if (!node_->configure()) {
        NODELET_FATAL("[gscam_cuda_pip][nodelet] configure() failed");
        node_.reset();
        return;
      }

      // initStream: GStreamer 파이프라인 구성 및 PLAY
      if (!node_->initStream()) {
        NODELET_FATAL("[gscam_cuda_pip][nodelet] initStream() failed");
        node_.reset();
        return;
      }

      // 프레임 루프는 별도 스레드에서 실행 (nodelet 블로킹 방지)
      worker_ = std::thread([this]() {
        try {
          node_->publishLoop();
        } catch (const std::exception& e) {
          ROS_ERROR_STREAM("[gscam_cuda_pip][nodelet] publishLoop exception: " << e.what());
        } catch (...) {
          ROS_ERROR("[gscam_cuda_pip][nodelet] publishLoop unknown exception");
        }
      });

      NODELET_INFO("[gscam_cuda_pip][nodelet] started");
    } catch (const std::exception& e) {
      NODELET_FATAL_STREAM("[gscam_cuda_pip][nodelet] init exception: " << e.what());
      node_.reset();
    } catch (...) {
      NODELET_FATAL("[gscam_cuda_pip][nodelet] init unknown exception");
      node_.reset();
    }
  }

private:
  std::unique_ptr<GscamCudaPipNode> node_;
  std::thread worker_;
};

} // namespace gscam_cuda_pip

PLUGINLIB_EXPORT_CLASS(gscam_cuda_pip::GscamCudaPipNodelet, nodelet::Nodelet)
