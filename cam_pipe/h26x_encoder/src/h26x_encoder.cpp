#include "h26x_encoder/h26x_encoder.hpp"
#include <sensor_msgs/image_encodings.h>
#include <gst/video/video.h>
#include <cstring>
#include <cmath>
#include <limits>
#include <cstdint>

namespace {

// --- helpers (CPP 전용 자유함수) ---

// IDR 여부 확인 (DELTA_UNIT 플래그가 없으면 key)
static bool is_keyframe(GstBuffer* buf) {
  return (GST_BUFFER_FLAG_IS_SET(buf, GST_BUFFER_FLAG_DELTA_UNIT) == 0);
}

// out_topic 마지막 컴포넌트를 /stats 로 교체
static std::string make_stats_topic_from(const std::string& out_topic) {
  if (out_topic.empty()) return std::string("encoded/stats");
  auto pos = out_topic.rfind('/');
  if (pos == std::string::npos) return std::string("encoded/stats");
  std::string base = out_topic.substr(0, pos);
  return base + "/stats";
}

// μs 변환
static inline uint64_t to_us(const ros::Time& t) {
  return static_cast<uint64_t>(t.sec) * 1000000ULL +
         static_cast<uint64_t>(t.nsec / 1000);
}

// GStreamer bus watch 콜백 (자유함수)
gboolean on_gst_bus(GstBus* /*bus*/, GstMessage* msg, gpointer /*user*/) {
  switch (GST_MESSAGE_TYPE(msg)) {
    case GST_MESSAGE_ERROR: {
      GError* err=nullptr; gchar* dbg=nullptr;
      gst_message_parse_error(msg, &err, &dbg);
      ROS_ERROR("GST ERROR: %s (debug: %s)", err?err->message:"?", dbg?dbg:"-");
      if (err) g_error_free(err);
      if (dbg) g_free(dbg);
      break;
    }
    case GST_MESSAGE_WARNING: {
      GError* err=nullptr; gchar* dbg=nullptr;
      gst_message_parse_warning(msg, &err, &dbg);
      ROS_WARN("GST WARN: %s (debug: %s)", err?err->message:"?", dbg?dbg:"-");
      if (err) g_error_free(err);
      if (dbg) g_free(dbg);
      break;
    }
    case GST_MESSAGE_STATE_CHANGED: {
      // 필요 시 상태 로그를 더 찍고 싶으면 여기 추가
      break;
    }
    default: break;
  }
  return TRUE;
}

} // anonymous namespace

namespace encoder {

bool H26xEncoderNode::haveFactory(const char* name) {
  GstElementFactory* f = gst_element_factory_find(name);
  if (f) { gst_object_unref(f); return true; }
  return false;
}

H26xEncoderNode::H26xEncoderNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
: nh_(nh), pnh_(pnh)
{
  // ---- params ----
  std::string mode_str = "auto";
  std::string fps_mode_str = "nominal";

  pnh_.param<std::string>("input_topic",       params_.input_topic,       params_.input_topic);
  pnh_.param<std::string>("camera_info_topic", params_.camera_info_topic, params_.camera_info_topic);
  pnh_.param<std::string>("codec",             params_.codec,             params_.codec);
  pnh_.param<int>("bitrate_bps",               params_.bitrate_bps,       params_.bitrate_bps);
  pnh_.param<int>("gop",                       params_.gop,               params_.gop);
  pnh_.param<std::string>("output_topic",      params_.output_topic,      params_.output_topic);
  pnh_.param<std::string>("mode",              mode_str,                  mode_str);
  pnh_.param<int>("fps_hint",                  fps_hint_,                 fps_hint_);
  pnh_.param<std::string>("fps_mode",          fps_mode_str,              fps_mode_str);
  pnh_.param<int>("fps_nominal",               fps_nominal_,              fps_nominal_);

  // 추가: stream_id 파라미터
  int stream_id_param = 0;
  pnh_.param<int>("stream_id", stream_id_param, 0);
  stream_id_ = static_cast<uint8_t>(stream_id_param);

  if (params_.codec != "h264" && params_.codec != "h265") {
    ROS_WARN("Unsupported codec '%s', fallback to h264", params_.codec.c_str());
    params_.codec = "h264";
  }
  if (mode_str == "cpu") params_.mode = Mode::CPU;
  else if (mode_str == "hw") params_.mode = Mode::HW;
  else params_.mode = Mode::AUTO;

  use_nominal_fps_ = (fps_mode_str == "nominal");

  if (params_.output_topic.empty()) {
    params_.output_topic = "encoded/" + params_.codec;
  }

  // 안전한 FPS 기본값
  if (fps_hint_ <= 0)    fps_hint_ = 30;
  if (fps_nominal_ <= 0) fps_nominal_ = 30;

  // ---- gstreamer init ----
  int argc = 0; char** argv = nullptr;
  gst_init(&argc, &argv);

  // ---- ROS pubs/subs ----
  pub_       = nh_.advertise<h26x_encoder::EncodedFrame>(params_.output_topic, 10);
  const std::string stats_topic = make_stats_topic_from(params_.output_topic);
  pub_stats_ = nh_.advertise<h26x_encoder::EncoderStats>(stats_topic, 1);
  stats_timer_ = nh_.createTimer(ros::Duration(1.0), &H26xEncoderNode::publishStats, this);
  
  sub_info_ = nh_.subscribe(params_.camera_info_topic, 1, &H26xEncoderNode::camInfoCb, this);
  sub_img_  = nh_.subscribe(params_.input_topic, 2, &H26xEncoderNode::imageCb, this);

  ROS_INFO("h26x_encoder: input=%s, cam_info=%s, codec=%s, bitrate=%d, gop=%d, mode=%s, out=%s, fps_mode=%s, fps_nominal=%d, fps_hint=%d, stream_id=%d, stats=%s",
           params_.input_topic.c_str(), params_.camera_info_topic.c_str(),
           params_.codec.c_str(), params_.bitrate_bps, params_.gop,
           (params_.mode==Mode::AUTO?"auto":params_.mode==Mode::CPU?"cpu":"hw"),
           params_.output_topic.c_str(),
           (use_nominal_fps_?"nominal":"measured"), fps_nominal_, fps_hint_,
           static_cast<int>(stream_id_),
           stats_topic.c_str());
}

H26xEncoderNode::~H26xEncoderNode() {
  destroyPipeline();
}

void H26xEncoderNode::camInfoCb(const sensor_msgs::CameraInfoConstPtr& info) {
  if (!have_dims_) {
    if (info->width > 0 && info->height > 0) {
      width_  = info->width;
      height_ = info->height;
      have_dims_ = true;
      ROS_INFO("Got camera info: %ux%u", width_.load(), height_.load());
      maybeBuildPipeline();
    }
  }
}

void H26xEncoderNode::maybeBuildPipeline() {
  std::lock_guard<std::mutex> lk(gst_mtx_);
  if (pipeline_ready_) return;
  if (!have_dims_) return;

  // decide mode
  Mode mode = params_.mode;
  if (mode == Mode::AUTO) {
    const bool have_hw_h264 = haveFactory("nvv4l2h264enc");
    const bool have_hw_h265 = haveFactory("nvv4l2h265enc");
    const bool ok = (params_.codec=="h264") ? have_hw_h264 : have_hw_h265;
    mode = ok ? Mode::HW : Mode::CPU; // Jetson이면 HW, 아니면 CPU
  }

  try {
    if (mode == Mode::HW) buildPipelineHw();
    else                  buildPipelineCpu();
    pipeline_ready_ = true;
    frame_seq_ = 0;
  } catch (const std::exception& e) {
    ROS_FATAL("Failed to build pipeline: %s", e.what());
    pipeline_ready_ = false;
  }
}

void H26xEncoderNode::buildPipelineCpu() {
  const auto w = width_.load();
  const auto h = height_.load();
  const int kbps = std::max(1, params_.bitrate_bps / 1000);

  const std::string enc =
    (params_.codec == "h265")
      ? "x265enc tune=zerolatency bitrate=" + std::to_string(kbps) +
        " key-int-max=" + std::to_string(std::max(1, params_.gop)) + " "
        "bframes=0 "
        "! h265parse config-interval=1 "
        "! video/x-h265,stream-format=(string)byte-stream,alignment=(string)au"
      : "x264enc tune=zerolatency speed-preset=ultrafast "
        "bitrate=" + std::to_string(kbps) + " "
        "key-int-max=" + std::to_string(std::max(1, params_.gop)) + " "
        "bframes=0 byte-stream=true "
        "option-string=\"slice-max-size=1000:cabac=0:aud=1:sync-lookahead=0\" "
        "! h264parse config-interval=1 "
        "! video/x-h264,stream-format=(string)byte-stream,alignment=(string)au";

  // x264는 I420 입력을 선호. videoconvert 뒤 caps로 강제.
  const std::string pipe =
    "appsrc name=appsrc is-live=true block=true format=time do-timestamp=true "
    "! video/x-raw,format=BGR,width=" + std::to_string(w) +
      ",height=" + std::to_string(h) +
      ",framerate=" + std::to_string(fps_hint_) + "/1 "
    "! queue leaky=downstream max-size-buffers=4 "
    "! videoconvert ! video/x-raw,format=I420 "
    "! " + enc + " "
    "! appsink name=appsink emit-signals=true sync=false max-buffers=2 drop=true";

  GError* err = nullptr;
  pipeline_ = gst_parse_launch(pipe.c_str(), &err);
  if (err) {
    std::string msg = err->message ? err->message : "unknown";
    g_error_free(err);
    throw std::runtime_error("gst_parse_launch(CPU) error: " + msg);
  }

  appsrc_  = GST_APP_SRC(gst_bin_get_by_name(GST_BIN(pipeline_), "appsrc"));
  appsink_ = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(pipeline_), "appsink"));
  if (!appsrc_ || !appsink_) throw std::runtime_error("appsrc/appsink not found");

  // appsrc caps 고정 (BGR/W/H/FPS) — 협상 안정화
  GstCaps* acaps = gst_caps_new_simple("video/x-raw",
      "format",    G_TYPE_STRING, "BGR",
      "width",     G_TYPE_INT,    (int)w,
      "height",    G_TYPE_INT,    (int)h,
      "framerate", GST_TYPE_FRACTION, fps_hint_, 1, NULL);
  gst_app_src_set_caps(appsrc_, acaps);
  gst_caps_unref(acaps);

  // appsink caps 명시 (alignment=au 확보)
  {
    const char* sink_caps = (params_.codec=="h265")
      ? "video/x-h265,stream-format=byte-stream,alignment=au"
      : "video/x-h264,stream-format=byte-stream,alignment=au";
    GstCaps* scaps = gst_caps_from_string(sink_caps);
    gst_app_sink_set_caps(appsink_, scaps);
    gst_caps_unref(scaps);
  }

  // new-sample 콜백 연결
  gst_app_sink_set_emit_signals(appsink_, true);
  g_signal_connect(appsink_, "new-sample",
                   G_CALLBACK(+[](GstAppSink* sink, gpointer user)->GstFlowReturn{
                     auto* self = static_cast<H26xEncoderNode*>(user);
                     if (auto* s = gst_app_sink_pull_sample(sink)) { self->publishSample(s); gst_sample_unref(s); }
                     return GST_FLOW_OK;
                   }), this);

  // bus watch
  GstBus* bus = gst_element_get_bus(pipeline_);
  gst_bus_add_watch(bus, on_gst_bus, this);
  gst_object_unref(bus);

  gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  ROS_INFO("GStreamer CPU pipeline ready (x264, I420, slice-max-size)");
}

void H26xEncoderNode::buildPipelineHw() {
  const auto w = width_.load();
  const auto h = height_.load();

  const bool have_h264 = haveFactory("nvv4l2h264enc");
  const bool have_h265 = haveFactory("nvv4l2h265enc");
  if ((params_.codec=="h264" && !have_h264) || (params_.codec=="h265" && !have_h265)) {
    throw std::runtime_error("Requested HW encoder not found (nvv4l2h26xenc)");
  }

  const std::string enc =
    (params_.codec == "h265")
      ? "nvv4l2h265enc insert-sps-pps=true control-rate=1 maxperf-enable=1 "
        "iframeinterval=" + std::to_string(std::max(1, params_.gop)) + " "
        "bitrate=" + std::to_string(params_.bitrate_bps) + " "
        "! h265parse config-interval=1 "
        "! video/x-h265,stream-format=(string)byte-stream,alignment=(string)au"
      : "nvv4l2h264enc insert-sps-pps=true control-rate=1 maxperf-enable=1 "
        "iframeinterval=" + std::to_string(std::max(1, params_.gop)) + " "
        "bitrate=" + std::to_string(params_.bitrate_bps) + " "
        "! h264parse config-interval=1 "
        "! video/x-h264,stream-format=(string)byte-stream,alignment=(string)au";

  // 변환 체인 간소화: nvvidconv로 NVMM/NV12 정렬
  const std::string pipe =
    "appsrc name=appsrc is-live=true block=true format=time do-timestamp=true "
    "! video/x-raw,format=BGR,width=" + std::to_string(w) +
      ",height=" + std::to_string(h) +
      ",framerate=" + std::to_string(fps_hint_) + "/1 "
    "! queue leaky=downstream max-size-buffers=4 "
    "! videoconvert ! video/x-raw, format=NV12 "
    "! nvvidconv ! video/x-raw(memory:NVMM),format=NV12"
    ",width=" + std::to_string(w) + ",height=" + std::to_string(h) + " "
    "! " + enc + " "
    "! appsink name=appsink emit-signals=true sync=false max-buffers=2 drop=true";

  GError* err = nullptr;
  pipeline_ = gst_parse_launch(pipe.c_str(), &err);
  if (err) {
    std::string msg = err->message ? err->message : "unknown";
    g_error_free(err);
    throw std::runtime_error("gst_parse_launch(HW) error: " + msg);
  }

  appsrc_  = GST_APP_SRC(gst_bin_get_by_name(GST_BIN(pipeline_), "appsrc"));
  appsink_ = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(pipeline_), "appsink"));
  if (!appsrc_ || !appsink_) throw std::runtime_error("appsrc/appsink not found");

  // appsrc caps 고정
  GstCaps* acaps = gst_caps_new_simple("video/x-raw",
      "format",    G_TYPE_STRING, "BGR",
      "width",     G_TYPE_INT,    (int)w,
      "height",    G_TYPE_INT,    (int)h,
      "framerate", GST_TYPE_FRACTION, fps_hint_, 1, NULL);
  gst_app_src_set_caps(appsrc_, acaps);
  gst_caps_unref(acaps);

  // appsink caps 명시 (alignment=au 확보)
  {
    GstCaps* scaps = gst_caps_from_string(
      params_.codec=="h265"
      ? "video/x-h265,stream-format=byte-stream,alignment=au"
      : "video/x-h264,stream-format=byte-stream,alignment=au");
    gst_app_sink_set_caps(appsink_, scaps);
    gst_caps_unref(scaps);
  }

  gst_app_sink_set_emit_signals(appsink_, true);
  g_signal_connect(appsink_, "new-sample",
                   G_CALLBACK(+[](GstAppSink* sink, gpointer user)->GstFlowReturn{
                     auto* self = static_cast<H26xEncoderNode*>(user);
                     if (auto* s = gst_app_sink_pull_sample(sink)) { self->publishSample(s); gst_sample_unref(s); }
                     return GST_FLOW_OK;
                   }), this);

  // bus watch
  GstBus* bus = gst_element_get_bus(pipeline_);
  gst_bus_add_watch(bus, on_gst_bus, this);
  gst_object_unref(bus);

  gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  ROS_INFO("GStreamer HW pipeline ready (nvv4l2%senc)", params_.codec=="h265"?"h265":"h264");
}

void H26xEncoderNode::destroyPipeline() {
  std::lock_guard<std::mutex> lk(gst_mtx_);
  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
  }
  pipeline_ = nullptr;
  appsrc_   = nullptr;
  appsink_  = nullptr;
  pipeline_ready_ = false;
  frame_seq_ = 0;
}

void H26xEncoderNode::publishSample(GstSample* sample) {
  GstBuffer* buffer = gst_sample_get_buffer(sample);
  if (!buffer) return;

  GstMapInfo map;
  if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) return;

  // 원본 입력 시각(ROS)도 보존은 하되,
  ros::Time src_stamp;
  { std::lock_guard<std::mutex> lk(stamp_mtx_); src_stamp = last_input_stamp_; }

  h26x_encoder::EncodedFrame msg;

  // header.stamp: 퍼블리시 시각 or 원본
  msg.header.stamp = src_stamp.isZero() ? ros::Time::now() : src_stamp;

  // ====== ★ 중요: PTS/DTS를 GST에서 가져오기 ======
  const GstClockTime pts = GST_BUFFER_PTS_IS_VALID(buffer) ? GST_BUFFER_PTS(buffer) : GST_CLOCK_TIME_NONE;
  // const GstClockTime dts = GST_BUFFER_DTS_IS_VALID(buffer) ? GST_BUFFER_DTS(buffer) : pts;

  // μs로 변환
  auto to_us_gst = [](GstClockTime t)->uint64_t {
    return (t == GST_CLOCK_TIME_NONE) ? 0ULL
                                      : static_cast<uint64_t>(gst_util_uint64_scale(t, 1000000, GST_SECOND));
  };
  msg.pts_us = to_us_gst(pts);

  // ====== 메타 ======
  msg.codec   = params_.codec;
  msg.width   = width_.load();
  msg.height  = height_.load();
  if (use_nominal_fps_) { msg.fps_num = std::max(1, fps_nominal_); msg.fps_den = 1; }
  else {
    const double fps = currentFps();
    if (fps > 0.1) { msg.fps_num = (uint32_t)std::round(fps*1000.0); msg.fps_den = 1000; }
    else           { msg.fps_num = (uint32_t)std::max(1, fps_hint_); msg.fps_den = 1;    }
  }

  msg.keyframe       = is_keyframe(buffer);
  msg.target_bitrate = params_.bitrate_bps;
  msg.stream_id      = stream_id_;
  msg.frame_id       = (uint32_t)frame_seq_.fetch_add(1, std::memory_order_relaxed);

  // Annex-B AU
  msg.data.assign(map.data, map.data + map.size);

  pub_.publish(msg);
  gst_buffer_unmap(buffer, &map);

  frames_total_.fetch_add(1, std::memory_order_relaxed);
  bytes_out_accum_.fetch_add(msg.data.size(), std::memory_order_relaxed);
}


void H26xEncoderNode::pushFrameToAppsrc(const uint8_t* data, size_t bytes, const ros::Time& /*stamp*/) {
  if (!pipeline_ready_ || !appsrc_) return;

  GstBuffer* buf = gst_buffer_new_allocate(nullptr, bytes, nullptr);
  GstMapInfo map;
  gst_buffer_map(buf, &map, GST_MAP_WRITE);
  std::memcpy(map.data, data, bytes);
  gst_buffer_unmap(buf, &map);

  // if (!stamp.isZero()) {
  //   const GstClockTime pts = gst_util_uint64_scale(stamp.toNSec(), 1, 1); // ns -> ns
  //   GST_BUFFER_PTS(buf) = pts;
  // }

  GstFlowReturn ret;
  g_signal_emit_by_name(appsrc_, "push-buffer", buf, &ret);
  gst_buffer_unref(buf);
  if (ret != GST_FLOW_OK) {
    ROS_WARN_THROTTLE(1.0, "appsrc push-buffer flow=%d", ret);
  }
}

void H26xEncoderNode::imageCb(const sensor_msgs::ImageConstPtr& msg) {
  const uint32_t w = msg->width, h = msg->height;
  bool need_rebuild = false;

  if (!have_dims_) {
    if (w>0 && h>0) { width_=w; height_=h; have_dims_=true; need_rebuild=true; }
    else return;
  } else {
    if (w != width_.load() || h != height_.load()) {
      width_ = w; height_ = h;
      need_rebuild = true;
      ROS_WARN("Resolution changed to %ux%u -> rebuilding pipeline", w, h);
    }
  }

  if (need_rebuild) {
    destroyPipeline();
    maybeBuildPipeline();
    if (!pipeline_ready_) return;
  }

  { std::lock_guard<std::mutex> lk(stamp_mtx_); last_input_stamp_ = msg->header.stamp; }

  // Push frame (BGR8 expected; otherwise convert)
  if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
    pushFrameToAppsrc(msg->data.data(), msg->step * msg->height, msg->header.stamp);
  } else {
    try {
      auto cv = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      pushFrameToAppsrc(cv->image.data, cv->image.total() * cv->image.elemSize(), msg->header.stamp);
    } catch (const std::exception& e) {
      ROS_WARN_THROTTLE(1.0, "cv_bridge conversion failed: %s", e.what());
      return;
    }
  }

  // FPS/지터 통계 갱신
  updateFpsEstimate(msg->header.stamp);
  updateStatsOnFrame(msg->header.stamp);
}

void H26xEncoderNode::updateFpsEstimate(const ros::Time& stamp) {
  std::lock_guard<std::mutex> lk(fps_mtx_);
  ts_queue_.push_back(stamp);
  if (ts_queue_.size() > fps_window_) ts_queue_.pop_front();
}

double H26xEncoderNode::currentFps() const {
  std::lock_guard<std::mutex> lk(fps_mtx_);
  if (ts_queue_.size() < 2) return 0.0;
  const ros::Time& first = ts_queue_.front();
  const ros::Time& last  = ts_queue_.back();
  const double dt = (last - first).toSec();
  if (dt <= 0.0) return 0.0;
  return static_cast<double>(ts_queue_.size() - 1) / dt;
}

void H26xEncoderNode::updateStatsOnFrame(const ros::Time& stamp) {
  if (!last_ts_.isZero()) {
    const double dt = (stamp - last_ts_).toSec();
    if (dt > 1e-6) {
      fps_instant_ = 1.0 / dt;
      if (ema_fps_ <= 0.0) ema_fps_ = fps_instant_;
      else                 ema_fps_ = ema_alpha_ * fps_instant_ + (1.0 - ema_alpha_) * ema_fps_;

      const double period_ms  = dt * 1000.0;
      const double nominal_ms = 1000.0 / static_cast<double>(std::max(1, fps_nominal_));
      const double err_ms     = period_ms - nominal_ms;
      jitter_sq_accum_ += (err_ms * err_ms);
      jitter_count_++;
      jitter_ms_rms_ = std::sqrt(jitter_sq_accum_ / std::max<uint32_t>(1, jitter_count_));
    }
  }
  last_ts_ = stamp;
}

void H26xEncoderNode::publishStats(const ros::TimerEvent&) {
  h26x_encoder::EncoderStats st;
  st.header.stamp    = ros::Time::now();
  st.fps_measured    = ema_fps_;
  st.fps_instant     = fps_instant_;
  st.jitter_ms_rms   = jitter_ms_rms_;
  st.frames_total    = static_cast<uint32_t>(std::min<uint64_t>(frames_total_.load(), std::numeric_limits<uint32_t>::max()));
  st.frames_dropped  = static_cast<uint32_t>(std::min<uint64_t>(frames_dropped_.load(), std::numeric_limits<uint32_t>::max()));
  pub_stats_.publish(st);
}

} // namespace encoder
