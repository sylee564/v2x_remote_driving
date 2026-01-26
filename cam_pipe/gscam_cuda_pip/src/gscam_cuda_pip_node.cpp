#include "gscam_cuda_pip/gscam_cuda_pip_node.hpp"
#include <cstring>
#include <algorithm>

namespace gscam_cuda_pip {

// ===== BUS 콜백 =====
gboolean GscamCudaPipNode::bus_watch_cb_(GstBus*, GstMessage* msg, gpointer) {
  switch (GST_MESSAGE_TYPE(msg)) {
    case GST_MESSAGE_ERROR: {
      GError* e=nullptr; gchar* dbg=nullptr;
      gst_message_parse_error(msg, &e, &dbg);
      ROS_ERROR("[gscam_cuda_pip] GST ERROR: %s (debug: %s)", e?e->message:"?", dbg?dbg:"-");
      if (e) g_error_free(e); if (dbg) g_free(dbg);
      break;
    }
    case GST_MESSAGE_WARNING: {
      GError* e=nullptr; gchar* dbg=nullptr;
      gst_message_parse_warning(msg, &e, &dbg);
      ROS_WARN("[gscam_cuda_pip] GST WARN: %s (debug: %s)", e?e->message:"?", dbg?dbg:"-");
      if (e) g_error_free(e); if (dbg) g_free(dbg);
      break;
    }
    default: break;
  }
  return TRUE;
}

// ===== ctor/dtor =====
GscamCudaPipNode::GscamCudaPipNode(ros::NodeHandle nh, ros::NodeHandle pnh)
: nh_(std::move(nh)), pnh_(std::move(pnh)), it_(nh_), cinfo_mgr_(nh_) {}

GscamCudaPipNode::~GscamCudaPipNode() { cleanupStream(); }

// ===== 파라미터 로드(전용) =====
void GscamCudaPipNode::loadPipParams_() {
  // 네임스페이스
  pnh_.param("ns", params_.ns, params_.ns);

  // PiP 관련 (최소치/상한 클램프 포함)
  pnh_.param("pip_enable",   params_.pip_enable,   params_.pip_enable);
  pnh_.param("pip_topic",    params_.pip_topic,    params_.pip_topic);
  pnh_.param("parking_topic",params_.parking_topic,params_.parking_topic);
  pnh_.param("pip_scale_w",  params_.pip_scale_w,  params_.pip_scale_w);
  pnh_.param("pip_scale_h",  params_.pip_scale_h,  params_.pip_scale_h);
  pnh_.param("pip_margin",   params_.pip_margin,   params_.pip_margin);
  pnh_.param("bg_dim_alpha", params_.bg_dim_alpha, params_.bg_dim_alpha);
  pnh_.param("draw_border",  params_.draw_border,  params_.draw_border);
  pnh_.param("border_px",    params_.border_px,    params_.border_px);

  std::string pos = "top_center";
  pnh_.param("pip_position", pos, pos);
  for (auto& c : pos) c = ::tolower(c);

  if (pos == "top_center" || pos == "topcentre" || pos == "top") {
    params_.pip_pos = GscamParams::PipPos::TopCenter;
  } else if (pos == "bottom_left" || pos == "bl") {
    params_.pip_pos = GscamParams::PipPos::BottomLeft;
  } else {
    params_.pip_pos = GscamParams::PipPos::BottomRight;
  }

  params_.pip_scale_w  = std::max(0.01, std::min(1.0, params_.pip_scale_w));
  params_.pip_scale_h  = std::max(0.01, std::min(1.0, params_.pip_scale_h));
  params_.pip_margin   = std::max(0, params_.pip_margin);
  params_.bg_dim_alpha = std::max(0.0, std::min(1.0, params_.bg_dim_alpha));
  params_.border_px    = std::max(0, params_.border_px);
}

// ===== configure =====
bool GscamCudaPipNode::configure() {
  // gscam_config
  std::string cfg_param;
  const bool has_param = pnh_.getParam("gscam_config", cfg_param);
  const char* cfg_env  = std::getenv("GSCAM_CONFIG");
  if (!cfg_env && !has_param) {
    ROS_FATAL("GSCAM_CONFIG env 또는 'gscam_config' 파라미터가 필요합니다.");
    return false;
  }
  if (cfg_env && has_param) {
    ROS_FATAL("GSCAM_CONFIG env와 'gscam_config' 파라미터를 동시에 설정할 수 없습니다.");
    return false;
  }
  params_.gsconfig = has_param ? cfg_param : std::string(cfg_env);
  ROS_INFO_STREAM("[gscam_cuda_pip] Using gstreamer config: \"" << params_.gsconfig << "\"");

  // 기타 파라미터
  pnh_.param("sync_sink",          params_.sync_sink,         params_.sync_sink);
  pnh_.param("preroll",            params_.preroll,           params_.preroll);
  pnh_.param("use_gst_timestamps", params_.use_gst_ts,        params_.use_gst_ts);
  pnh_.param("reopen_on_eof",      params_.reopen_on_eof,     params_.reopen_on_eof);

  pnh_.param("camera_info_url",    params_.camera_info_url,   params_.camera_info_url);
  pnh_.param("camera_name",        params_.camera_name,       params_.camera_name);
  pnh_.param("frame_id",           params_.frame_id,          params_.frame_id);

  pnh_.param("image_encoding",     params_.image_encoding,    params_.image_encoding);
  pnh_.param("use_rectify",        params_.use_rectify,       params_.use_rectify);

  // PiP/parking 파라미터
  loadPipParams_();

  // CameraInfo
  cinfo_mgr_.setCameraName(params_.camera_name);
  if (!params_.camera_info_url.empty() && cinfo_mgr_.validateURL(params_.camera_info_url)) {
    cinfo_mgr_.loadCameraInfo(params_.camera_info_url);
    ROS_INFO_STREAM("[gscam_cuda_pip] Loaded camera_info: " << params_.camera_info_url);
  } else {
    if (!params_.camera_info_url.empty())
      ROS_WARN_STREAM("[gscam_cuda_pip] Invalid camera_info_url: " << params_.camera_info_url);
  }

  // Publishers
  pub_raw_     = it_.advertise("image_raw", 1);
  pub_overlay_ = it_.advertise("image_overlay", 1);
  pub_rect_    = it_.advertiseCamera("image_rect", 1);

  // Subscribers
  image_transport::TransportHints hints("compressed", ros::TransportHints().tcpNoDelay());

  if (params_.pip_enable && !params_.pip_topic.empty()) {
    pip_sub_ = it_.subscribe(params_.pip_topic, 1, &GscamCudaPipNode::cbPip_, this, hints);
    ROS_INFO_STREAM("[gscam_cuda_pip] PiP subscribed: " << params_.pip_topic);
  }

  // front에서만 parking_sub_ 사용
  if (params_.ns == "front" && !params_.parking_topic.empty()) {
    parking_sub_ = nh_.subscribe(params_.parking_topic, 1,
                               &GscamCudaPipNode::cbParking_, this,
                               ros::TransportHints().tcpNoDelay());
    // parking_sub_ = it_.subscribe(params_.parking_topic, 1, &GscamCudaPipNode::cbParking_, this, hints);
    ROS_INFO_STREAM("[gscam_cuda_pip] Parking subscribed: " << params_.parking_topic);
  }

  // Control subscriber
  sub_ctrl_ = nh_.subscribe("control_cmd", 1, &GscamCudaPipNode::cbControl_, this);

  return true;
}

// ===== appsink caps (BGR) =====
void GscamCudaPipNode::setAppSinkCaps_() {
  if (!sink_) return;
  GstCaps* caps = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, "BGR", NULL);
  gst_app_sink_set_caps(GST_APP_SINK(sink_), caps);
  gst_caps_unref(caps);
}

// ===== appsink 연결 =====
bool GscamCudaPipNode::attachSinkToPipeline_(GstElement* pipeline) {
  if (!pipeline) return false;

  sink_ = gst_element_factory_make("appsink", nullptr);
  if (!sink_) {
    ROS_FATAL("[gscam_cuda_pip] appsink create failed");
    return false;
  }
  g_object_set(G_OBJECT(sink_),
               "emit-signals", FALSE,
               "sync", (params_.sync_sink ? TRUE : FALSE),
               "max-buffers", 1,
               "drop", TRUE,
               "enable-last-sample", FALSE,
               NULL);
  setAppSinkCaps_();

  if (GST_IS_PIPELINE(pipeline)) {
    GstPad* outpad = gst_bin_find_unlinked_pad(GST_BIN(pipeline), GST_PAD_SRC);
    if (!outpad) {
      ROS_FATAL("[gscam_cuda_pip] No unlinked SRC pad in pipeline");
      return false;
    }
    GstElement* outelem = gst_pad_get_parent_element(outpad);
    gst_object_unref(outpad);
    if (!outelem) {
      ROS_FATAL("[gscam_cuda_pip] Failed to get parent element for SRC pad.");
      return false;
    }
    if (!gst_bin_add(GST_BIN(pipeline), sink_)) {
      ROS_FATAL("[gscam_cuda_pip] gst_bin_add(sink) failed");
      gst_object_unref(outelem);
      return false;
    }
    if (!gst_element_link(outelem, sink_)) {
      ROS_FATAL("[gscam_cuda_pip] cannot link %s -> appsink", gst_element_get_name(outelem));
      gst_object_unref(outelem);
      return false;
    }
    gst_object_unref(outelem);
  } else {
    // 단일 element인 경우 새 파이프라인을 만들어 연결
    GstElement* launchpipe = pipeline;
    pipeline_ = gst_pipeline_new(nullptr);
    if (!pipeline_) {
      ROS_FATAL("[gscam_cuda_pip] gst_pipeline_new failed");
      return false;
    }
    gst_object_unparent(GST_OBJECT(launchpipe));
    gst_bin_add_many(GST_BIN(pipeline_), launchpipe, sink_, NULL);
    if (!gst_element_link(launchpipe, sink_)) {
      ROS_FATAL("[gscam_cuda_pip] cannot link launchpipe -> appsink");
      return false;
    }
  }
  return true;
}

// ===== initStream =====
bool GscamCudaPipNode::initStream() {
  if (!gst_is_initialized()) gst_init(nullptr, nullptr);
  ROS_INFO_STREAM("[gscam_cuda_pip] GStreamer " << gst_version_string());

  GError* error = nullptr;
  pipeline_ = gst_parse_launch(params_.gsconfig.c_str(), &error);
  if (!pipeline_) {
    ROS_FATAL_STREAM("[gscam_cuda_pip] gst_parse_launch failed: " << (error? error->message : "unknown"));
    if (error) g_error_free(error);
    return false;
  }

  if (!attachSinkToPipeline_(pipeline_)) return false;

  // BUS
  {
    GstBus* bus = gst_element_get_bus(pipeline_);
    gst_bus_add_watch(bus, &GscamCudaPipNode::bus_watch_cb_, nullptr);
    gst_object_unref(bus);
  }

  if (params_.preroll) {
    gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    gst_element_get_state(pipeline_, nullptr, nullptr, -1);
    gst_element_set_state(pipeline_, GST_STATE_PAUSED);
    gst_element_get_state(pipeline_, nullptr, nullptr, -1);
  }

  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    ROS_ERROR("[gscam_cuda_pip] Could not start stream (PLAY)");
    return false;
  }
  ROS_INFO("[gscam_cuda_pip] Started stream.");

  // 시간 오프셋(참고)
  GstClock* clock = gst_system_clock_obtain();
  ros::Time now = ros::Time::now();
  GstClockTime ct = gst_clock_get_time(clock);
  gst_object_unref(clock);
  time_offset_ = now.toSec() - GST_TIME_AS_USECONDS(ct)/1e6;
  ROS_INFO_STREAM("[gscam_cuda_pip] Time offset: " << time_offset_);
  return true;
}

// ===== 타임스탬프 변환 =====
ros::Time GscamCudaPipNode::stampFromGst_(GstBuffer* buf) const {
  if (!params_.use_gst_ts || !pipeline_ || !buf) return ros::Time::now();
  GstClockTime bt  = gst_element_get_base_time(pipeline_);
  GstClockTime pts = buf->pts;
  if (pts == GST_CLOCK_TIME_NONE) return ros::Time::now();
  const double t = (double)GST_TIME_AS_USECONDS(pts + bt) / 1e6;
  return ros::Time(t);
}

// ===== 크기 질의 =====
bool GscamCudaPipNode::querySizeFromSinkPad_(int& w, int& h) const {
  if (!sink_) return false;
  GstPad* pad = gst_element_get_static_pad(sink_, "sink");
  if (!pad) return false;
#if GST_VERSION_MAJOR == 1
  const GstCaps* caps = gst_pad_get_current_caps(pad);
#else
  const GstCaps* caps = gst_pad_get_negotiated_caps(pad);
#endif
  gst_object_unref(pad);
  if (!caps) return false;
  GstStructure* s = gst_caps_get_structure(caps, 0);
  if (!s) return false;
  if (!gst_structure_get_int(s, "width", &w))  return false;
  if (!gst_structure_get_int(s, "height", &h)) return false;
  return true;
}

// ===== rectify 맵 1회 준비 =====
void GscamCudaPipNode::prepareRectifyMapsOnce_(int W, int H) {
  if (maps_ready_ || !params_.use_rectify) return;

  const auto info = cinfo_mgr_.getCameraInfo();
  if (info.K.size()!=9 || info.D.empty()) {
    ROS_WARN_THROTTLE(2.0, "[gscam_cuda_pip] camera_info invalid");
    return;
  }

  cv::Mat K = (cv::Mat1d(3,3) <<
    info.K[0], info.K[1], info.K[2],
    info.K[3], info.K[4], info.K[5],
    info.K[6], info.K[7], info.K[8]);
  cv::Mat D(info.D);
  cv::Mat R = cv::Mat::eye(3,3,CV_64F);
  cv::Mat P = (cv::Mat1d(3,4) <<
    info.P[0], info.P[1], info.P[2],  info.P[3],
    info.P[4], info.P[5], info.P[6],  info.P[7],
    info.P[8], info.P[9], info.P[10], info.P[11]);
  cv::Mat P3 = P(cv::Rect(0,0,3,3)).clone();

  cv::initUndistortRectifyMap(K, D, R, P3, cv::Size(W,H),
                              CV_32FC1, mapx_cpu_, mapy_cpu_);
  mapx_gpu_.upload(mapx_cpu_, stream_);
  mapy_gpu_.upload(mapy_cpu_, stream_);
  stream_.waitForCompletion();

  maps_ready_ = true;
  ROS_INFO_STREAM("[gscam_cuda_pip] rectify maps uploaded: " << W << "x" << H);
}

// ===== 발행 헬퍼 =====
void GscamCudaPipNode::publishBGR_(const cv::Mat& bgr,
                                   const image_transport::Publisher& pub,
                                   const std_msgs::Header& h) {
  if (bgr.empty() || !pub) return;
  sensor_msgs::Image msg;
  msg.header = h;
  msg.width  = bgr.cols;
  msg.height = bgr.rows;
  msg.encoding = sensor_msgs::image_encodings::BGR8;
  msg.is_bigendian = false;
  msg.step = static_cast<uint32_t>(bgr.step);
  msg.data.assign(bgr.datastart, bgr.dataend);
  pub.publish(msg);
}

void GscamCudaPipNode::publishBGRwithInfo_(const cv::Mat& bgr,
                                           const image_transport::CameraPublisher& pub,
                                           const sensor_msgs::CameraInfo& cinfo) {
  if (bgr.empty() || !pub) return;
  sensor_msgs::ImagePtr img(new sensor_msgs::Image());
  img->header = cinfo.header;
  img->width  = bgr.cols;
  img->height = bgr.rows;
  img->encoding = sensor_msgs::image_encodings::BGR8;
  img->is_bigendian = false;
  img->step = static_cast<uint32_t>(bgr.step);
  img->data.assign(bgr.datastart, bgr.dataend);
  pub.publish(img, boost::make_shared<sensor_msgs::CameraInfo>(cinfo));
}

// ===== PiP 콜백 =====
void GscamCudaPipNode::cbPip_(const sensor_msgs::ImageConstPtr& msg) {
  try {
    cv_bridge::CvImageConstPtr cvp = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    cv::cuda::GpuMat g;
    g.upload(cvp->image, stream_);
    std::lock_guard<std::mutex> lk(pip_mtx_);
    g.copyTo(g_pip_latest_, stream_);
  } catch (const cv_bridge::Exception& e) {
    ROS_WARN_STREAM("[gscam_cuda_pip] pip cv_bridge: " << e.what());
  }
}

// ===== Parking 콜백 =====
// void GscamCudaPipNode::cbParking_(const sensor_msgs::ImageConstPtr& msg) {
//   try {
//     cv_bridge::CvImageConstPtr cvp = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
//     cv::cuda::GpuMat g;
//     g.upload(cvp->image, stream_);
//     std::lock_guard<std::mutex> lk(pip_parking_mtx_);
//     g.copyTo(g_parking_latest_, stream_);
//   } catch (const cv_bridge::Exception& e) {
//     ROS_WARN_STREAM("[gscam_cuda_pip] parking cv_bridge: " << e.what());
//   }
// }

void GscamCudaPipNode::cbParking_(const sensor_msgs::CompressedImageConstPtr& msg) {
  try {
    if (!msg || msg->data.empty()) return;

    cv::Mat buf(1, static_cast<int>(msg->data.size()), CV_8UC1,
                const_cast<uint8_t*>(msg->data.data()));
    cv::Mat bgr = cv::imdecode(buf, cv::IMREAD_COLOR);
    if (bgr.empty()) {
      ROS_WARN_THROTTLE(2.0, "[gscam_cuda_pip] parking imdecode failed (format=%s)",
                        msg->format.c_str());
      return;
    }

    cv::cuda::GpuMat g;
    g.upload(bgr, stream_);
    std::lock_guard<std::mutex> lk(pip_parking_mtx_);
    g.copyTo(g_parking_latest_, stream_);
  } catch (const std::exception& e) {
    ROS_WARN_STREAM("[gscam_cuda_pip] parking compressed decode: " << e.what());
  }
}


// ===== Control 콜백 =====
void GscamCudaPipNode::cbControl_(const tod_msgs::ControlCmdConstPtr& msg) {
  gear_.store(msg->shift.data, std::memory_order_relaxed);
  indicator_.store(msg->indicator.data, std::memory_order_relaxed);
  parking_flag_.store(msg->parking_flag, std::memory_order_relaxed);
}

// ===== 위치별 rect 계산 (pip_position 반영) =====
cv::Rect GscamCudaPipNode::pipRect_(int W, int H, int w, int h, int margin) const {
  int x = 0, y = 0;

  switch (params_.pip_pos) {
    case GscamParams::PipPos::TopCenter:
      x = std::max(0, (W - w) / 2);
      y = std::max(0, margin);
      break;

    case GscamParams::PipPos::BottomLeft:
      x = std::max(0, margin);
      y = std::max(0, H - h - margin);
      break;

    case GscamParams::PipPos::BottomRight:
    default:
      x = std::max(0, W - w - margin);
      y = std::max(0, H - h - margin);
      break;
  }

  // clamp
  x = std::min(x, std::max(0, W - w));
  y = std::min(y, std::max(0, H - h));

  return cv::Rect(x, y, w, h);
}

// ===== 공통 오버레이 (pip_position 반영) =====
void GscamCudaPipNode::overlayPip_(cv::cuda::GpuMat& canvas_bgr, const cv::cuda::GpuMat& pip_bgr) {
  if (pip_bgr.empty()) return;

  const int W = canvas_bgr.cols, H = canvas_bgr.rows;

  const int tw = std::min(std::max(1, int(std::round(W * params_.pip_scale_w))), W);
  const int th = std::min(std::max(1, int(std::round(H * params_.pip_scale_h))), H);

  cv::cuda::GpuMat resized;
  cv::cuda::resize(pip_bgr, resized, cv::Size(tw, th), 0, 0, cv::INTER_AREA, stream_);

  const cv::Rect r = pipRect_(W, H, tw, th, params_.pip_margin);
  cv::cuda::GpuMat roi(canvas_bgr, r);

  if (params_.bg_dim_alpha > 0.0) {
    cv::cuda::multiply(roi, cv::Scalar::all(1.0 - params_.bg_dim_alpha), roi, 1.0, -1, stream_);
  }

  resized.copyTo(roi, stream_);

  if (params_.draw_border && params_.border_px > 0) {
    const int t = std::min({params_.border_px, r.width, r.height});
    const cv::Scalar col(200,200,200);
    cv::cuda::GpuMat top(canvas_bgr, cv::Rect(r.x, r.y, r.width, t));                 top.setTo(col, stream_);
    cv::cuda::GpuMat bot(canvas_bgr, cv::Rect(r.x, r.y + r.height - t, r.width, t));  bot.setTo(col, stream_);
    cv::cuda::GpuMat lef(canvas_bgr, cv::Rect(r.x, r.y, t, r.height));                lef.setTo(col, stream_);
    cv::cuda::GpuMat rig(canvas_bgr, cv::Rect(r.x + r.width - t, r.y, t, r.height));  rig.setTo(col, stream_);
  }
}

// ===== Front 전용 전환 로직 =====
void GscamCudaPipNode::overlayPipOnCanvas_Front_(cv::cuda::GpuMat& canvas_bgr,
                                                 const cv::cuda::GpuMat& front_rect_snapshot)
{
  if (params_.ns != "front") return;

  // 최신 rear(sub) / parking 스냅샷 가져오기
  cv::cuda::GpuMat g_rear, g_parking;
  { std::lock_guard<std::mutex> lk(pip_mtx_);         if (!g_pip_latest_.empty())      g_rear = g_pip_latest_; }
  { std::lock_guard<std::mutex> lk(pip_parking_mtx_); if (!g_parking_latest_.empty())  g_parking = g_parking_latest_; }

  const bool has_rear    = !g_rear.empty();
  const bool has_parking = !g_parking.empty();
  const bool has_front   = !front_rect_snapshot.empty();

  const bool is_reverse = (gear_.load(std::memory_order_relaxed) == eGearPosition::GEARPOSITION_REVERSE);
  const bool parking    = parking_flag_.load(std::memory_order_relaxed);

  // rear는 기존처럼 flip
  cv::cuda::GpuMat g_rear_flipped;
  if (has_rear) {
    cv::cuda::flip(g_rear, g_rear_flipped, 1, stream_);  // 1 == horizontal flip
  }

  if (is_reverse && has_rear) {
    // 메인 = rear
    if (g_rear_flipped.size() != canvas_bgr.size()) {
      cv::cuda::resize(g_rear_flipped, canvas_bgr, canvas_bgr.size(), 0, 0, cv::INTER_LINEAR, stream_);
    } else {
      g_rear_flipped.copyTo(canvas_bgr, stream_);
    }

    // PiP = front rect snapshot (pip_position 반영)
    if (has_front) overlayPip_(canvas_bgr, front_rect_snapshot);
    return;
  }

  // Drive: 메인 = front(rect) (canvas_bgr는 이미 front rect 기반)
  // PiP = parking_flag면 parking을 우선, 아니면 rear
  if (parking && has_parking) {
    overlayPip_(canvas_bgr, g_parking);
  } else if (has_rear) {
    overlayPip_(canvas_bgr, g_rear_flipped);
  }
}

// ===== Left/Right 전용: indicator gate + pip_position =====
void GscamCudaPipNode::overlayPipOnCanvas_Side_(cv::cuda::GpuMat& canvas_bgr)
{
  if (params_.ns != "left" && params_.ns != "right") return;

  // indicator gate (요구사항: 1=left, 2=right)
  const int ind = indicator_.load(std::memory_order_relaxed);
  if (params_.ns == "left") {
    if (ind != 1) return;
  } else {
    if (ind != 2) return;
  }

  // latest pip snapshot
  cv::cuda::GpuMat g_pip;
  { std::lock_guard<std::mutex> lk(pip_mtx_);
    if (!g_pip_latest_.empty()) g_pip = g_pip_latest_;
  }
  if (g_pip.empty()) return;

  // 위치는 pip_position을 따름
  overlayPip_(canvas_bgr, g_pip);
}

// ===== publishLoop =====
void GscamCudaPipNode::publishLoop() {
  ROS_INFO("[gscam_cuda_pip] Publishing loop start...");
  while (ros::ok()) {
#if GST_VERSION_MAJOR == 1
    if (!pipeline_ || !sink_) break;

    GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(sink_));
    if (!sample) { ROS_INFO("[gscam_cuda_pip] EOS/pull failed"); break; }

    GstBuffer*  buf  = gst_sample_get_buffer(sample);
    const GstCaps* caps = gst_sample_get_caps(sample);

    GstVideoInfo vinfo;
    if (!caps || !gst_video_info_from_caps(&vinfo, caps)) {
      gst_sample_unref(sample); ros::spinOnce(); continue;
    }

    GstVideoFrame vframe;
    if (!gst_video_frame_map(&vframe, &vinfo, buf, GST_MAP_READ)) {
      gst_sample_unref(sample); ros::spinOnce(); continue;
    }

    const int W = GST_VIDEO_FRAME_WIDTH(&vframe);
    const int H = GST_VIDEO_FRAME_HEIGHT(&vframe);
    uint8_t*   data = (uint8_t*)GST_VIDEO_FRAME_PLANE_DATA(&vframe, 0);
    const int  step = GST_VIDEO_FRAME_PLANE_STRIDE(&vframe, 0);
    cv::Mat cpu_bgr(H, W, CV_8UC3, data, step);

    // 타임스탬프/CameraInfo
    sensor_msgs::CameraInfo cinfo = cinfo_mgr_.getCameraInfo();
    cinfo.header.stamp    = stampFromGst_(buf);
    cinfo.header.frame_id = params_.frame_id;

    // /image_raw
    publishBGR_(cpu_bgr, pub_raw_, cinfo.header);

    // rectify 준비
    if (params_.use_rectify) prepareRectifyMapsOnce_(W, H);

    // GPU 업로드 → rect
    g_in_.create(H, W, CV_8UC3);
    g_in_.upload(cpu_bgr, stream_);

    if (params_.use_rectify && !mapx_gpu_.empty()) {
      g_rect_.create(H, W, CV_8UC3);
      cv::cuda::remap(g_in_, g_rect_, mapx_gpu_, mapy_gpu_,
                      cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(), stream_);
    } else {
      g_rect_ = g_in_;
    }

    // Front rect 스냅샷(Deep copy)
    cv::cuda::GpuMat g_front_rect_snapshot;
    if (params_.ns == "front") {
      g_rect_.copyTo(g_front_rect_snapshot, stream_);
    }

    // Overlay 시작: 기본 canvas는 rect
    g_rect_.copyTo(g_overlay_, stream_);

    // 오버레이 적용 (호출은 항상 해도 내부 ns gate로 no-op)
    overlayPipOnCanvas_Front_(g_overlay_, g_front_rect_snapshot);
    overlayPipOnCanvas_Side_(g_overlay_);

    // 다운로드 & 발행
    stream_.waitForCompletion();
    cv::Mat rect_cpu, overlay_cpu;
    if (!g_rect_.empty())    g_rect_.download(rect_cpu);
    if (!g_overlay_.empty()) g_overlay_.download(overlay_cpu);

    // /image_rect (with CameraInfo)
    publishBGRwithInfo_(rect_cpu,    pub_rect_,    cinfo);

    // /image_overlay
    publishBGR_(overlay_cpu,         pub_overlay_, cinfo.header);

    gst_video_frame_unmap(&vframe);
    gst_sample_unref(sample);
#else
    break;
#endif
    ros::spinOnce();
  }
}

// ===== stop/cleanup =====
void GscamCudaPipNode::stopPipeline_() {
  if (pipeline_) gst_element_set_state(pipeline_, GST_STATE_NULL);
}

void GscamCudaPipNode::cleanupStream() {
  stopPipeline_();
  if (pipeline_) { gst_object_unref(pipeline_); pipeline_ = nullptr; }
  sink_ = nullptr;
}

void GscamCudaPipNode::run() {
  while (ros::ok()) {
    if (!configure()) { ROS_FATAL("[gscam_cuda_pip] configure() failed"); break; }
    if (!initStream()) { ROS_FATAL("[gscam_cuda_pip] initStream() failed"); break; }
    publishLoop();
    cleanupStream();
    ROS_INFO("[gscam_cuda_pip] Stream stopped.");
    if (params_.reopen_on_eof) {
      ROS_INFO("[gscam_cuda_pip] Reopening stream...");
    } else break;
  }
}

} // namespace gscam_cuda_pip
