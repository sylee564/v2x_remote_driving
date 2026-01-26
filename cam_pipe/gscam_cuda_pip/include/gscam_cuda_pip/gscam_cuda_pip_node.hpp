#pragma once

// ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>

// Control enums
#include <tod_msgs/VehicleEnums.h>
#include <tod_msgs/ControlCmd.h>

// STL
#include <mutex>
#include <atomic>
#include <string>

// OpenCV (CUDA)
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudawarping.hpp>

// GStreamer
extern "C" {
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>
#include <gst/video/gstvideometa.h>
}

namespace gscam_cuda_pip {

// ===== 사용자 파라미터 =====
struct GscamParams {
  // GStreamer
  std::string gsconfig;              // gscam_config (env GSCAM_CONFIG도 허용)
  bool        sync_sink        = true;
  bool        preroll          = false;
  bool        use_gst_ts       = true;
  bool        reopen_on_eof    = false;

  // CameraInfo
  std::string camera_info_url;
  std::string camera_name      = "camera";
  std::string frame_id         = "camera_frame";

  // Encoding
  std::string image_encoding   = sensor_msgs::image_encodings::BGR8;

  // Rectify
  bool        use_rectify      = true;

  // ns (front / left / right / rear)
  std::string ns               = "front";

  // PiP 파라미터 (Rear/Side 오버레이용)
  bool        pip_enable       = true;
  std::string pip_topic        = "/rear/camera/image_raw"; // rear or side wide source
  std::string parking_topic    = "/rear/parking_image";    // parking source (front R+parking)
  double      pip_scale_w      = 0.5;
  double      pip_scale_h      = 0.2;
  int         pip_margin       = 12;
  double      bg_dim_alpha     = 0.18;
  bool        draw_border      = true;
  int         border_px        = 2;

  enum class PipPos { TopCenter, BottomLeft, BottomRight };
  PipPos      pip_pos          = PipPos::TopCenter;
};

// ===== 노드 클래스 =====
class GscamCudaPipNode {
public:
  explicit GscamCudaPipNode(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~GscamCudaPipNode();

  // 메인 루프
  void run();

  // 단계별
  bool configure();        // 파라미터/Publisher/Subscriber 설정
  bool initStream();       // GStreamer 파이프라인 구성
  void publishLoop();      // 프레임 루프 (CUDA 처리 및 발행)
  void cleanupStream();    // 파이프라인 정리

private:
  // ===== ROS =====
  ros::NodeHandle nh_, pnh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher      pub_raw_;
  image_transport::Publisher      pub_overlay_;
  image_transport::CameraPublisher pub_rect_;
  camera_info_manager::CameraInfoManager cinfo_mgr_;

  // Subscribers
  image_transport::Subscriber pip_sub_;       // rear/side image
  // image_transport::Subscriber parking_sub_;   // parking image (front 용)
  ros::Subscriber parking_sub_;
  ros::Subscriber             sub_ctrl_;      // control_cmd (gear/indicator/parking)

  // ===== 파라미터/상태 =====
  GscamParams params_;
  int width_  = 0;
  int height_ = 0;

  // ===== GStreamer =====
  GstElement* pipeline_ = nullptr;
  GstElement* sink_     = nullptr;  // appsink
  double      time_offset_ = 0.0;

  // ===== CUDA / OpenCV =====
  cv::cuda::Stream stream_;
  // remap maps
  bool maps_ready_ = false;
  cv::Mat mapx_cpu_, mapy_cpu_;
  cv::cuda::GpuMat mapx_gpu_, mapy_gpu_;
  // GPU work buffers
  cv::cuda::GpuMat g_in_, g_rect_, g_overlay_;
  // PiP/parking 최신 프레임
  std::mutex pip_mtx_, pip_parking_mtx_;
  cv::cuda::GpuMat g_pip_latest_;      // rear/side
  cv::cuda::GpuMat g_parking_latest_;  // parking

  // Control states
  std::atomic<int>  gear_{eGearPosition::GEARPOSITION_PARK};
  std::atomic<int>  indicator_{eIndicator::INDICATOR_OFF};
  std::atomic<bool> parking_flag_{false};

  // ===== 내부 유틸 =====
  // 파라미터 로드
  void loadPipParams_();

  // appsink caps 설정
  void setAppSinkCaps_();

  // appsink를 파이프라인에 붙이고 링크
  bool attachSinkToPipeline_(GstElement* pipeline);

  // GStreamer 버스 메시지(에러/워닝) 로그
  static gboolean bus_watch_cb_(GstBus*, GstMessage*, gpointer);

  // GStreamer 타임스탬프 → ROS Time
  ros::Time stampFromGst_(GstBuffer* buf) const;

  // 현재 caps에서 width/height 추출
  bool querySizeFromSinkPad_(int& w, int& h) const;

  // 안전한 정지
  void stopPipeline_();

  // CameraInfo 로드/리매핑 준비(1회)
  void prepareRectifyMapsOnce_(int W, int H);

  // 발행 헬퍼
  void publishBGR_(const cv::Mat& bgr, const image_transport::Publisher& pub, const std_msgs::Header& h);
  void publishBGRwithInfo_(const cv::Mat& bgr, const image_transport::CameraPublisher& pub, const sensor_msgs::CameraInfo& cinfo);

  // PiP/parking 콜백
  void cbPip_(const sensor_msgs::ImageConstPtr& msg);      // rear/side
  // void cbParking_(const sensor_msgs::ImageConstPtr& msg);  // parking (front)
  void cbParking_(const sensor_msgs::CompressedImageConstPtr& msg);
  void cbControl_(const tod_msgs::ControlCmdConstPtr& msg);

  // ===== 오버레이 배치/렌더 =====
  // 위치별 rect 계산 (pip_position 반영)
  cv::Rect pipRect_(int W, int H, int w, int h, int margin) const;

  // 공통 오버레이 (pip_position 반영)
  void overlayPip_(cv::cuda::GpuMat& canvas_bgr, const cv::cuda::GpuMat& pip_bgr);

  // Front 전용 전환 로직 (rear/parking + gear)
  void overlayPipOnCanvas_Front_(cv::cuda::GpuMat& canvas_bgr,
                                 const cv::cuda::GpuMat& front_rect_snapshot);

  // Left/Right 전용: indicator gate(1=left, 2=right) + pip_position
  void overlayPipOnCanvas_Side_(cv::cuda::GpuMat& canvas_bgr);
};

} // namespace gscam_cuda_pip
