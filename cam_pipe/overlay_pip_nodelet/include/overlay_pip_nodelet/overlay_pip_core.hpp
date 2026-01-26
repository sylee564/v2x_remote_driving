#pragma once
#include <mutex>
#include <string>
#include <atomic>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// 기어 상태 구독 메시지 (필드명 shift.data 사용). 패키지 경로는 환경에 맞게 조정하세요.
#include <tod_msgs/ControlCmd.h>
#include <tod_msgs/VehicleEnums.h>

namespace overlay_pip_nodelet {

class OverlayPipCore {
public:
  OverlayPipCore() = default;
  ~OverlayPipCore() = default;

  // 노드/노드렛 공통 초기화
  void init(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
  // ====== 설정 파라미터 (모두 pnh에서 읽음) ======
  std::string name_space_{"front"};
  double pip_scale_w_ = 0.5;
  double pip_scale_h_ = 0.2;
  int    pip_margin_  = 0;
  int    queue_size_  = 1;

  // ★ 추가: 배경/오버레이 알파, 테두리
  double bg_dim_alpha_    = 0.25;   // ROI 배경을 어둡게 하는 비율 (0~1), 0.25면 25% 어둡게
  double overlay_alpha_   = 1.0;    // PiP 자체의 가중치 (0~1). 1.0이면 완전 불투명 copy와 유사
  bool   draw_border_     = true;   // 테두리 on/off
  int    border_thickness_= 2;      // 테두리 두께(px)

  std::string control_topic_ = "/vehicle/control_cmd";

  // ====== ROS IO ======
  std::unique_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber sub_main_;
  image_transport::Subscriber sub_sub_;
  image_transport::Subscriber sub_parking_;
  image_transport::Publisher  pub_out_;
  ros::Subscriber             sub_control_;

  // ====== 상태 ======
  std::mutex       mtx_sub_;
  std::mutex       mtx_parking_;
  cv::Mat          sub_latest_;         // 최신 sub rgb8
  cv::Mat          parking_latest_;
  std_msgs::Header sub_latest_header_;  // rear 헤더(스위치 시 사용)
  std_msgs::Header parking_latest_header_;  // rear 헤더(스위치 시 사용)
  std::atomic<uint8_t> gear_state_{0};       // 기어 상태 
  std::atomic<uint8_t> indicator_state_{0};       // 방향지시등 상태
  std::atomic<bool> parking_state_{false};       // 방향지시등 상태

  // ====== 콜백 ======
  void cbControlCmd(const tod_msgs::ControlCmdConstPtr& msg);
  void cbSubImage(const sensor_msgs::ImageConstPtr& msg);
  void cbParkingImage(const sensor_msgs::ImageConstPtr& msg);
  void cbMainImage(const sensor_msgs::ImageConstPtr& msg);

  // ====== 오버레이 유틸 ======
  enum class OverlayPos { TopCenter, BottomRight, BottomLeft };
  void overlayAt(cv::Mat& canvas_bgr, const cv::Mat& pip_bgr, OverlayPos pos);
  inline cv::Rect makeRect_(int W, int H, int w, int h, OverlayPos pos) const;
};

} // namespace overlay_pip_nodelet
