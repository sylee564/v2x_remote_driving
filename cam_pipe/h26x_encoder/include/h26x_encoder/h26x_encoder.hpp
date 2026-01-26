#pragma once
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <h26x_encoder/EncodedFrame.h>
#include <h26x_encoder/EncoderStats.h>
#include <cv_bridge/cv_bridge.h>
#include <tod_msgs/ControlCmd.h>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>

#include <mutex>
#include <deque>
#include <string>
#include <atomic>
#include <cmath>
#include <limits>
#include <cstdint>

namespace encoder {

// 인코딩 모드
enum class Mode { AUTO, CPU, HW };

// 런타임 파라미터
struct EncoderParams {
  std::string input_topic       = "/front/image_raw";
  std::string camera_info_topic = "/front/camera_info";
  std::string codec             = "h264";     // "h264" | "h265"
  Mode        mode              = Mode::AUTO; // auto | cpu | hw
  int         bitrate_bps       = 2'500'000;  // 2.5 Mbps
  int         gop               = 30;         // IDR every ~1s @30fps
  std::string output_topic;                   // empty -> "encoded/<codec>"
};

// H.26x 인코더 노드
class H26xEncoderNode {
public:
  explicit H26xEncoderNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~H26xEncoderNode();

  // 복사/이동 금지(ROS pub/sub + GStreamer 핸들 수명 보호)
  H26xEncoderNode(const H26xEncoderNode&) = delete;
  H26xEncoderNode& operator=(const H26xEncoderNode&) = delete;
  H26xEncoderNode(H26xEncoderNode&&) = delete;
  H26xEncoderNode& operator=(H26xEncoderNode&&) = delete;

private:
  // ==== ROS Callbacks ====
  void imageCb(const sensor_msgs::ImageConstPtr& msg);
  void camInfoCb(const sensor_msgs::CameraInfoConstPtr& info);

  // ==== Pipeline lifecycle ====
  void maybeBuildPipeline();
  void buildPipelineCpu();                // x264enc/x265enc
  void buildPipelineHw();                 // Jetson: nvv4l2h26xenc
  void destroyPipeline();

  // ==== GStreamer helpers ====
  void publishSample(GstSample* sample);  // appsink → EncodedFrame publish
  void pushFrameToAppsrc(const uint8_t* data, size_t bytes, const ros::Time& stamp);
  static bool haveFactory(const char* name);

  // ==== Timing / FPS ====
  void   updateFpsEstimate(const ros::Time& stamp);  // 슬라이딩 윈도 FPS 추정
  double currentFps() const;

  // ==== Stats ====
  void updateStatsOnFrame(const ros::Time& stamp);   // 입력 프레임 도착 시
  void publishStats(const ros::TimerEvent&);         // 1Hz 타이머 콜백으로 퍼블리시

  // ==== State ====
  ros::NodeHandle nh_, pnh_;
  EncoderParams   params_;

  // Topics
  ros::Subscriber sub_img_;
  ros::Subscriber sub_info_;
  ros::Publisher  pub_;        // EncodedFrame
  ros::Publisher  pub_stats_;  // EncoderStats
  ros::Timer      stats_timer_;

  // 입력 스탬프
  ros::Time last_input_stamp_;
  std::mutex stamp_mtx_;

  // Stream 식별자(멀티 카메라 지원) — 파라미터로 설정
  uint8_t stream_id_{0};

  // Camera info
  std::atomic<uint32_t> width_{0}, height_{0};
  std::atomic<bool>     have_dims_{false};

  // FPS/Nominal params
  // - EncodedFrame에는 기본적으로 nominal FPS를 넣음 (use_nominal_fps_=true)
  // - 측정치를 쓰고 싶으면 파라미터로 false 설정
  bool use_nominal_fps_{true};
  int  fps_nominal_{30};     // EncodedFrame용 명세 FPS
  int  fps_hint_{30};        // caps 협상용 기본 FPS

  // FPS estimator (window-based, 입력 기준)
  mutable std::mutex fps_mtx_;
  std::deque<ros::Time> ts_queue_;
  size_t fps_window_ = 60;

  // Stats (EMA & jitter) — 입력 콜백에서만 갱신, 퍼블리시는 타이머
  double ema_fps_{0.0};           // 측정 FPS의 EMA
  double ema_alpha_{0.2};         // EMA 계수(필요 시 파라미터화)
  double fps_instant_{0.0};       // 직전 프레임 간격 기반 FPS
  double jitter_ms_rms_{0.0};     // RMS 지터(ms)

  // 내부 누적용
  double    jitter_sq_accum_{0.0};
  uint32_t  jitter_count_{0};
  ros::Time last_ts_;             // 즉시 FPS/지터 계산용

  // Counters (GStreamer 스레드/ROS 콜백 동시 접근 가능 → atomic)
  // frame_seq_는 EncodedFrame.frame_id 생성에 사용(32-bit로 캐스팅)
  std::atomic<uint64_t> frame_seq_{0};
  std::atomic<uint64_t> frames_total_{0};
  std::atomic<uint64_t> frames_dropped_{0};  // 필요 시 구현부에서 증가
  std::atomic<uint64_t> bytes_out_accum_{0}; // 최근 구간 전송 바이트(옵션)

  // GStreamer (GStreamer 콜백은 별도 스레드)
  GstElement* pipeline_{nullptr};
  GstAppSrc*  appsrc_{nullptr};
  GstAppSink* appsink_{nullptr};
  std::mutex  gst_mtx_;
  std::atomic<bool> pipeline_ready_{false};
};

} // namespace encoder