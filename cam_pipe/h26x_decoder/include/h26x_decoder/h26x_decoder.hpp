#pragma once
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <h26x_encoder/EncodedFrame.h>
#include <h26x_encoder/EncoderStats.h>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>

#include <mutex>
#include <string>
#include <atomic>
#include <deque>
#include <vector>
#include <cstring>
#include <cmath>

namespace h26xdec {

enum class Mode { AUTO, CPU, HW };

struct DecoderParams {
  std::string input_topic  = "encoded/h264";   // EncodedFrame
  std::string output_topic = "image_decoded";  // sensor_msgs::Image(BGR8)
  std::string codec        = "auto";           // auto|h264|h265 (msg.codec 우선)
  Mode        mode         = Mode::AUTO;       // auto|cpu|hw
  bool        wait_for_idr = true;             // 시작 시 IDR 전까지 드롭
  int         fps_hint     = 30;               // PTS/DUR 힌트(없을 때)
  std::string stats_topic  = "";               // 비면 output_topic 기준 자동 결정
};

class H26xDecoderNode {
public:
  explicit H26xDecoderNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~H26xDecoderNode();

  // 복사/이동 금지(노드/노드렛 수명 안전)
  H26xDecoderNode(const H26xDecoderNode&) = delete;
  H26xDecoderNode& operator=(const H26xDecoderNode&) = delete;
  H26xDecoderNode(H26xDecoderNode&&) = delete;
  H26xDecoderNode& operator=(H26xDecoderNode&&) = delete;

private:
  // ROS
  void frameCb(const h26x_encoder::EncodedFrameConstPtr& msg);

  // Pipeline
  void maybeBuild(const std::string& codec);
  void buildCpu(const std::string& codec);     // avdec_h26x
  void buildHw (const std::string& codec);     // nvv4l2decoder
  void destroy();

  // Helpers
  static bool haveFactory(const char* name);
  void pushAuToAppsrc(const uint8_t* data, size_t bytes, double fps,
                      const ros::Time& stamp, bool keyframe);
  void publishSample(GstSample* sample);
  void publishStats(const ros::TimerEvent&);

  // FPS utils (입력 기반)
  void   updateFpsEstimate(const ros::Time& ts);
  double currentFps() const;
  double jitterRmsMs() const;

  // State
  ros::NodeHandle nh_, pnh_;
  DecoderParams params_;

  ros::Subscriber sub_;
  ros::Publisher  pub_;        // /.../decoded/image
  ros::Publisher  pub_stats_;  // /.../decoded/stats
  ros::Timer      stats_timer_;

  // 입력 해상도 힌트(디코더가 caps로 알 수 있지만 로깅/메타용)
  std::atomic<uint32_t> width_{0}, height_{0};

  // 키프레임 게이트
  std::atomic<bool>     started_{false};     // 첫 IDR 이후 true
  std::atomic<uint64_t> frame_seq_{0};       // PTS 증가용 시퀀스

  // 통계(입력/출력 타임스탬프)
  mutable std::mutex fps_mtx_;
  std::deque<ros::Time> tsq_;      // 입력 AU 도착 시각
  std::deque<ros::Time> out_tsq_;  // 출력 이미지 퍼블리시 시각
  size_t fps_window_{60};

  std::atomic<uint64_t> frames_in_{0};
  std::atomic<uint64_t> frames_out_{0};
  std::atomic<uint64_t> bytes_in_accum_{0};  // 통계용 누적 바이트

  // GStreamer
  GstElement* pipeline_{nullptr};
  GstAppSrc*  appsrc_{nullptr};
  GstAppSink* appsink_{nullptr};
  std::mutex  gst_mtx_;
  std::atomic<bool> ready_{false};
  std::string active_codec_{"h264"};  // h264|h265
  Mode        active_mode_{Mode::CPU};
};

} // namespace h26xdec
