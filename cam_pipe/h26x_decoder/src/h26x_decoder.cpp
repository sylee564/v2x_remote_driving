#include "h26x_decoder/h26x_decoder.hpp"
#include <limits>
#include <vector>

namespace h26xdec {

static std::string to_mode_str(Mode m){
  return (m==Mode::AUTO?"auto":(m==Mode::CPU?"cpu":"hw"));
}

bool H26xDecoderNode::haveFactory(const char* name) {
  GstElementFactory* f = gst_element_factory_find(name);
  if (f) { gst_object_unref(f); return true; }
  return false;
}

H26xDecoderNode::H26xDecoderNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
: nh_(nh), pnh_(pnh)
{
  std::string mode_str = "auto";
  pnh_.param<std::string>("input_topic",  params_.input_topic,  params_.input_topic);
  pnh_.param<std::string>("output_topic", params_.output_topic, params_.output_topic);
  pnh_.param<std::string>("codec",        params_.codec,        params_.codec);
  pnh_.param<std::string>("mode",         mode_str,             mode_str);
  pnh_.param<bool>("wait_for_idr",        params_.wait_for_idr, params_.wait_for_idr);
  pnh_.param<int>("fps_hint",             params_.fps_hint,     params_.fps_hint);
  pnh_.param<std::string>("stats_topic",  params_.stats_topic,  params_.stats_topic);

  if      (mode_str=="cpu") active_mode_ = Mode::CPU;
  else if (mode_str=="hw")  active_mode_ = Mode::HW;
  else                      active_mode_ = Mode::AUTO;

  // GStreamer init
  int argc=0; char** argv=nullptr;
  gst_init(&argc, &argv);

  pub_ = nh_.advertise<sensor_msgs::Image>(params_.output_topic, 1);
  sub_ = nh_.subscribe(params_.input_topic, 30, &H26xDecoderNode::frameCb, this);

  // stats topic 자동 생성
  std::string stats_topic = params_.stats_topic;
  if (stats_topic.empty()) {
    stats_topic = params_.output_topic;
    const std::string suffix = "/image";
    if (!stats_topic.empty() &&
        stats_topic.size() >= suffix.size() &&
        stats_topic.compare(stats_topic.size()-suffix.size(), suffix.size(), suffix)==0) {
      stats_topic.replace(stats_topic.size()-suffix.size(), suffix.size(), "/stats");
    } else {
      if (!stats_topic.empty() && stats_topic.back()!='/') stats_topic += "/";
      stats_topic += "stats";
    }
  }
  pub_stats_   = nh_.advertise<h26x_encoder::EncoderStats>(stats_topic, 1);
  stats_timer_ = nh_.createTimer(ros::Duration(1.0), &H26xDecoderNode::publishStats, this);

  ROS_INFO("h26x_decoder: in=%s out=%s codec=%s mode=%s wait_for_idr=%s fps_hint=%d",
           params_.input_topic.c_str(), params_.output_topic.c_str(),
           params_.codec.c_str(), to_mode_str(active_mode_).c_str(),
           params_.wait_for_idr?"true":"false", params_.fps_hint);
}

H26xDecoderNode::~H26xDecoderNode() { destroy(); }

void H26xDecoderNode::destroy() {
  std::lock_guard<std::mutex> lk(gst_mtx_);
  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
  }
  pipeline_ = nullptr;
  appsrc_   = nullptr;
  appsink_  = nullptr;
  ready_    = false;
  started_  = false;
  frame_seq_= 0;
}

void H26xDecoderNode::maybeBuild(const std::string& codec) {
  std::lock_guard<std::mutex> lk(gst_mtx_);
  if (ready_) return;

  // codec 결정
  active_codec_ = codec;
  if (active_codec_!="h264" && active_codec_!="h265") {
    active_codec_ = "h264"; // fallback
  }

  // mode 자동 판단
  Mode mode = active_mode_;
  if (mode==Mode::AUTO) {
    const bool have_hw = haveFactory("nvv4l2decoder");
    mode = have_hw ? Mode::HW : Mode::CPU;
  }

  try {
    if (mode==Mode::HW) buildHw(active_codec_);
    else                buildCpu(active_codec_);
    ready_ = true;
    active_mode_ = mode;
  } catch (const std::exception& e) {
    ROS_FATAL("Failed to build decoder pipeline: %s", e.what());
    ready_ = false;
  }
}

void H26xDecoderNode::buildCpu(const std::string& codec) {
  const std::string parse = (codec=="h265") ? "h265parse"    : "h264parse";
  const std::string dec   = (codec=="h265") ? "avdec_h265"   : "avdec_h264";

  // capsfilter는 문자열에서 제거, appsrc caps는 코드로 "고정값" 설정
  const std::string pipe =
    "appsrc name=appsrc is-live=true block=true format=time do-timestamp=false "
    "! " + parse + " config-interval=-1 "
    "! " + dec + " "
    "! videoconvert ! video/x-raw,format=BGR "
    "! appsink name=appsink emit-signals=true sync=false max-buffers=2 drop=true";

  GError* err=nullptr;
  pipeline_ = gst_parse_launch(pipe.c_str(), &err);
  if (err) {
    std::string m = err->message ? err->message : "unknown";
    g_error_free(err);
    throw std::runtime_error("gst_parse_launch(CPU) error: " + m);
  }
  appsrc_  = GST_APP_SRC(gst_bin_get_by_name(GST_BIN(pipeline_), "appsrc"));
  appsink_ = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(pipeline_), "appsink"));
  if (!appsrc_ || !appsink_) throw std::runtime_error("appsrc/appsink not found");

  // ★ 고정 caps: byte-stream + AU (AVC 허용 X; 인코더에서 byte-stream 보냄)
  std::string caps_str = (codec=="h265")
    ? "video/x-h265, stream-format=(string)byte-stream, alignment=(string)au"
    : "video/x-h264, stream-format=(string)byte-stream, alignment=(string)au";
  GstCaps* acaps = gst_caps_from_string(caps_str.c_str());
  gst_app_src_set_caps(appsrc_, acaps);
  gst_caps_unref(acaps);

  gst_app_sink_set_emit_signals(appsink_, true);
  g_signal_connect(appsink_, "new-sample",
                   G_CALLBACK(+[](GstAppSink* sink, gpointer u)->GstFlowReturn {
                     auto* self = static_cast<H26xDecoderNode*>(u);
                     if (auto* s = gst_app_sink_pull_sample(sink)) { self->publishSample(s); gst_sample_unref(s); }
                     return GST_FLOW_OK;
                   }), this);

  gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  ROS_INFO("GStreamer CPU decoder ready (%s)", dec.c_str());
}

void H26xDecoderNode::buildHw(const std::string& codec) {
  // Jetson: nvv4l2decoder + nvvidconv → BGR
  if (!haveFactory("nvv4l2decoder"))
    throw std::runtime_error("nvv4l2decoder not found (HW mode requested)");

  const std::string parse = (codec=="h265") ? "h265parse"    : "h264parse";

  const std::string pipe =
    "appsrc name=appsrc is-live=true block=true format=time do-timestamp=false "
    "! " + parse + " config-interval=-1 "
    "! nvv4l2decoder "
    "! nvvidconv ! video/x-raw,format=BGRx "
    "! videoconvert ! video/x-raw,format=BGR "
    "! appsink name=appsink emit-signals=true sync=false max-buffers=2 drop=true";

  GError* err=nullptr;
  pipeline_ = gst_parse_launch(pipe.c_str(), &err);
  if (err) {
    std::string m = err->message ? err->message : "unknown";
    g_error_free(err);
    throw std::runtime_error("gst_parse_launch(HW) error: " + m);
  }
  appsrc_  = GST_APP_SRC(gst_bin_get_by_name(GST_BIN(pipeline_), "appsrc"));
  appsink_ = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(pipeline_), "appsink"));
  if (!appsrc_ || !appsink_) throw std::runtime_error("appsrc/appsink not found");

  // ★ 고정 caps: byte-stream + AU (HW도 동일 정책)
  std::string caps_str = (codec=="h265")
    ? "video/x-h265, stream-format=(string)byte-stream, alignment=(string)au"
    : "video/x-h264, stream-format=(string)byte-stream, alignment=(string)au";
  GstCaps* acaps = gst_caps_from_string(caps_str.c_str());
  gst_app_src_set_caps(appsrc_, acaps);
  gst_caps_unref(acaps);

  gst_app_sink_set_emit_signals(appsink_, true);
  g_signal_connect(appsink_, "new-sample",
                   G_CALLBACK(+[](GstAppSink* sink, gpointer u)->GstFlowReturn {
                     auto* self = static_cast<H26xDecoderNode*>(u);
                     if (auto* s = gst_app_sink_pull_sample(sink)) { self->publishSample(s); gst_sample_unref(s); }
                     return GST_FLOW_OK;
                   }), this);

  gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  ROS_INFO("GStreamer HW decoder ready (nvv4l2decoder)");
}

void H26xDecoderNode::updateFpsEstimate(const ros::Time& ts) {
  std::lock_guard<std::mutex> lk(fps_mtx_);
  tsq_.push_back(ts);
  if (tsq_.size() > fps_window_) tsq_.pop_front();
}

double H26xDecoderNode::currentFps() const {
  std::lock_guard<std::mutex> lk(fps_mtx_);
  if (tsq_.size()<2) return 0.0;
  const double dt = (tsq_.back()-tsq_.front()).toSec();
  if (dt<=0) return 0.0;
  return (tsq_.size()-1)/dt;
}

double H26xDecoderNode::jitterRmsMs() const {
  std::lock_guard<std::mutex> lk(fps_mtx_);
  if (tsq_.size() < 3) return 0.0;
  // dt 리스트
  std::vector<double> dts;
  dts.reserve(tsq_.size()-1);
  for (size_t i=1; i<tsq_.size(); ++i) {
    dts.push_back((tsq_[i] - tsq_[i-1]).toSec());
  }
  double mean = 0.0;
  for (double v: dts) mean += v;
  mean /= std::max<size_t>(1, dts.size());
  double acc = 0.0;
  for (double v: dts) {
    const double e = (v - mean) * 1000.0;
    acc += e*e;
  }
  return std::sqrt(acc / std::max<size_t>(1, dts.size()));
}

void H26xDecoderNode::publishStats(const ros::TimerEvent&) {
  h26x_encoder::EncoderStats st;
  st.header.stamp = ros::Time::now();

  // 입력 FPS(평활/인스턴트)
  const double fps_smoothed = currentFps();
  double fps_inst = 0.0;
  {
    std::lock_guard<std::mutex> lk(fps_mtx_);
    if (tsq_.size() >= 2) {
      const double dt = (tsq_.back() - tsq_.at(tsq_.size()-2)).toSec();
      if (dt > 1e-6) fps_inst = 1.0 / dt;
    }
  }
  st.fps_measured  = fps_smoothed;
  st.fps_instant   = fps_inst;
  st.jitter_ms_rms = jitterRmsMs();

  // 프레임 카운터
  const uint64_t fin  = frames_in_.load(std::memory_order_relaxed);
  const uint64_t fout = frames_out_.load(std::memory_order_relaxed);
  st.frames_total   = static_cast<uint32_t>(std::min<uint64_t>(fout, std::numeric_limits<uint32_t>::max()));
  const uint64_t drops = (fin > fout) ? (fin - fout) : 0;
  st.frames_dropped = static_cast<uint32_t>(std::min<uint64_t>(drops, std::numeric_limits<uint32_t>::max()));

  pub_stats_.publish(st);
  // bytes_in_accum_는 필요시 kbps 산출용으로 확장 가능 (fetch_add 사용 중)
}

void H26xDecoderNode::pushAuToAppsrc(const uint8_t* data, size_t bytes, double fps,
                                     const ros::Time& /*stamp*/, bool keyframe) {
  if (!ready_ || !appsrc_) return;

  // 시작 모드: IDR 전까지 드롭(옵션)
  if (params_.wait_for_idr && !started_) {
    if (!keyframe) return;
    started_ = true;
    frame_seq_ = 0;
  }

  GstBuffer* buf = gst_buffer_new_allocate(nullptr, bytes, nullptr);
  GstMapInfo map;
  gst_buffer_map(buf, &map, GST_MAP_WRITE);
  std::memcpy(map.data, data, bytes);
  gst_buffer_unmap(buf, &map);

  // PTS/DURATION (fps 우선순위: msg → 추정 → hint)
  if (fps <= 0.1) {
    double est = currentFps();
    fps = (est>0.1) ? est : static_cast<double>(params_.fps_hint);
  }
  const gint64 dur = (fps>1e-6)? static_cast<gint64>(GST_SECOND / fps) : GST_SECOND/30;
  const uint64_t id = frame_seq_.fetch_add(1, std::memory_order_relaxed);

  GST_BUFFER_PTS(buf)      = id * dur;
  GST_BUFFER_DURATION(buf) = dur;

  // keyframe 힌트
  if (keyframe) {
    GST_BUFFER_FLAG_UNSET(buf, GST_BUFFER_FLAG_DELTA_UNIT);
  } else {
    GST_BUFFER_FLAG_SET(buf,   GST_BUFFER_FLAG_DELTA_UNIT);
  }

  GstFlowReturn ret;
  g_signal_emit_by_name(appsrc_, "push-buffer", buf, &ret);
  gst_buffer_unref(buf);

  if (ret != GST_FLOW_OK) {
    ROS_WARN_THROTTLE(1.0, "appsrc push-buffer flow=%d", ret);
  }
}

void H26xDecoderNode::publishSample(GstSample* sample) {
  GstBuffer* buffer = gst_sample_get_buffer(sample);
  GstCaps* caps     = gst_sample_get_caps(sample);

  GstVideoInfo vinfo;
  if (!caps || !gst_video_info_from_caps(&vinfo, caps)) {
    ROS_WARN_THROTTLE(1.0, "publishSample: no caps or failed to parse caps");
    return;
  }
  const int w = GST_VIDEO_INFO_WIDTH(&vinfo);
  const int h = GST_VIDEO_INFO_HEIGHT(&vinfo);

  GstMapInfo map;
  if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) return;

  sensor_msgs::Image img;
  img.header.stamp   = ros::Time::now(); // 필요시 입력 stamp 맵핑 가능
  img.height         = h;
  img.width          = w;
  img.encoding       = sensor_msgs::image_encodings::BGR8;
  img.is_bigendian   = false;
  img.step           = w * 3;
  img.data.resize(img.step * h);
  std::memcpy(img.data.data(), map.data, img.data.size());

  gst_buffer_unmap(buffer, &map);
  pub_.publish(img);

  {
    std::lock_guard<std::mutex> lk(fps_mtx_);
    out_tsq_.push_back(img.header.stamp);
    if (out_tsq_.size() > fps_window_) out_tsq_.pop_front();
  }
  frames_out_.fetch_add(1, std::memory_order_relaxed);
}

void H26xDecoderNode::frameCb(const h26x_encoder::EncodedFrameConstPtr& msg) {
  // codec 결정(파라미터 auto면 msg.codec 사용)
  ROS_DEBUG("Got EncodedFrame: %s key=%d size=%zu",
            msg->codec.c_str(), (int)msg->keyframe, msg->data.size());

  std::string codec = params_.codec;
  if (codec=="auto" || codec.empty()) {
    codec = msg->codec;
  }
  if (codec != "h264" && codec != "h265") codec = "h264";

  // 파이프라인 준비
  maybeBuild(codec);

  // 폭/높이 힌트 갱신
  if (msg->width>0 && msg->height>0) {
    width_  = msg->width;
    height_ = msg->height;
  }

  // fps 계산(명세값)
  double fps = 0.0;
  if (msg->fps_den>0) fps = static_cast<double>(msg->fps_num) / static_cast<double>(msg->fps_den);

  // AU 푸시
  pushAuToAppsrc(msg->data.data(), msg->data.size(), fps, msg->header.stamp, msg->keyframe);

  // 입력 fps 추정(디버그용)
  updateFpsEstimate(msg->header.stamp);
  frames_in_.fetch_add(1, std::memory_order_relaxed);
  bytes_in_accum_.fetch_add(msg->data.size(), std::memory_order_relaxed);
}

} // namespace h26xdec
