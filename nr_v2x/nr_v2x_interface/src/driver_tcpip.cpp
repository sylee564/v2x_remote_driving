#include "nr_v2x_interface/driver_tcpip.hpp"

#include <ros/console.h>
#include <algorithm>
#include <cstring>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <arpa/inet.h>
#include <boost/asio/steady_timer.hpp>

using boost::asio::ip::udp;

namespace v2x_interface {

// ----------------------- ctor / dtor -----------------------
driver_tcpip_t::driver_tcpip_t(ros::NodeHandle& nh, ros::NodeHandle& pnh)
: nh_(nh)
, pnh_(pnh)
, nh_video_(nh)
, nh_data_(nh)
, m_work_guard(boost::asio::make_work_guard(m_io_context))
{
  // ----- CallbackQueue 분리 -----
  nh_video_.setCallbackQueue(&video_queue_);
  nh_data_.setCallbackQueue(&data_queue_);

  // ---------- params ----------
  {
    int v = static_cast<int>(video_frag_bytes_);
    pnh_.param<int>("video_frag_bytes", v, v);
    if (v < 256) v = 256;
    if (v > 1200) v = 1200;  // 최대 1200바이트까지
    video_frag_bytes_ = static_cast<uint32_t>(v);
  }
  {
    int v = static_cast<int>(video_pacing_us_);
    pnh_.param<int>("video_pacing_us", v, v);
    if (v < 0) v = 0;
    video_pacing_us_ = static_cast<uint32_t>(v);
  }

  { int v = static_cast<int>(psid_ext_);  pnh_.param<int>("psid_ext",  v, v); psid_ext_  = static_cast<uint32_t>(v); }
  { int v = static_cast<int>(cast_mode_); pnh_.param<int>("cast_mode", v, v); cast_mode_ = static_cast<uint8_t>(v); }
  { int v = static_cast<int>(src_id_);    pnh_.param<int>("src_id",    v, v); src_id_    = static_cast<uint32_t>(v); }
  { int v = static_cast<int>(dst_id_);    pnh_.param<int>("dst_id",    v, v); dst_id_    = static_cast<uint32_t>(v); }

  pnh_.param<std::string>("overall_version", overall_s_, std::string("v1"));
  if (overall_s_ != "v1" && overall_s_ != "v2") overall_s_ = "v1";

  pnh_.param<bool>("debug_rx_raw", debug_rx_raw_, false);
  pnh_.param<int>("build_threads", build_threads_, 2);
  if (build_threads_ < 1) build_threads_ = 1;
  build_pool_.reset(new boost::asio::thread_pool(build_threads_));

  // DB_V2X meta
  int dev_type=0, tcom=0, svc=0, act=0, region=0, ptype=0, comm=0;
  pnh_.param<int>("db_eDeviceType",   dev_type, dev_type);
  pnh_.param<int>("db_eTeleCommType", tcom,     tcom);
  pnh_.param<int>("db_eServiceId",    svc,      svc);
  pnh_.param<int>("db_eActionType",   act,      act);
  pnh_.param<int>("db_eRegionId",     region,   region);
  pnh_.param<int>("db_ePayloadType",  ptype,    ptype);
  pnh_.param<int>("db_eCommId",       comm,     comm);

  { int v = static_cast<int>(db_meta_.unDeviceId); pnh_.param<int>("db_unDeviceId", v, v); db_meta_.unDeviceId = static_cast<uint32_t>(v); }
  { int v = static_cast<int>(db_meta_.usDbVer);    pnh_.param<int>("db_usDbVer",    v, v); db_meta_.usDbVer    = static_cast<uint16_t>(v); }
  { int v = static_cast<int>(db_meta_.usHwVer);    pnh_.param<int>("db_usHwVer",    v, v); db_meta_.usHwVer    = static_cast<uint16_t>(v); }
  { int v = static_cast<int>(db_meta_.usSwVer);    pnh_.param<int>("db_usSwVer",    v, v); db_meta_.usSwVer    = static_cast<uint16_t>(v); }

  pnh_.param<bool>("isVehicle", isVehicle_, false);
  ROS_INFO_STREAM("isVehicle=" << (isVehicle_ ? "true" : "false"));

  db_meta_.eDeviceType   = static_cast<DB_V2X_DEVICE_TYPE_t>(dev_type);
  db_meta_.eTeleCommType = static_cast<DB_V2X_TELECOMMUNICATION_TYPE_t>(tcom);
  db_meta_.eServiceId    = static_cast<DB_V2X_SERVICE_ID_t>(svc);
  db_meta_.eActionType   = static_cast<DB_V2X_ACTION_TYPE_t>(act);
  db_meta_.eRegionId     = static_cast<DB_V2X_REGION_ID_t>(region);
  db_meta_.ePayloadType  = static_cast<DB_V2X_PAYLOAD_TYPE_t>(ptype);
  db_meta_.eCommId       = static_cast<DB_V2X_COMMUNCATION_ID_t>(comm);

  // ---------- topics ----------
  topic_video_[0] = "/front/encoded/h264";
  topic_video_[1] = "/left/encoded/h264";
  topic_video_[2] = "/right/encoded/h264";
  topic_video_[3] = "/avm/encoded/h264";
  pnh_.param<std::string>("topic_video_front", topic_video_[0], topic_video_[0]);
  pnh_.param<std::string>("topic_video_left",  topic_video_[1], topic_video_[1]);
  pnh_.param<std::string>("topic_video_right", topic_video_[2], topic_video_[2]);
  pnh_.param<std::string>("topic_video_avm",   topic_video_[3], topic_video_[3]);
  pnh_.param<std::string>("topic_tx_",        topic_tx_,        topic_tx_);

  // ---------- ROS IO ----------
  if (isVehicle_) {
    topic_tx_  = "/Vehicle/kona/interface/probe_vehicle_data";
    sub_tx     = nh_data_.subscribe(topic_tx_, 10,
                  &driver_tcpip_t::tx_callback<tod_msgs::ProbeVehicleData::ConstPtr>, this);
    pub_rx_payload_ = nh_data_.advertise<tod_msgs::ControlCmd>("control_cmd_data", 10);

    sub_videos_.resize(kMaxStreams);
    // 수신 지연 최소화: 큐 2, 영상 콜백은 video_queue_ 전용
    sub_videos_[0] = nh_video_.subscribe(topic_video_[0], 2, &driver_tcpip_t::callback_video0, this);
    sub_videos_[1] = nh_video_.subscribe(topic_video_[1], 2, &driver_tcpip_t::callback_video1, this);
    sub_videos_[2] = nh_video_.subscribe(topic_video_[2], 2, &driver_tcpip_t::callback_video2, this);
    sub_videos_[3] = nh_video_.subscribe(topic_video_[3], 2, &driver_tcpip_t::callback_video3, this);

    sub_tod_status  = nh_data_.subscribe("Vehicle/Manager/tod_status", 10,
                                         &driver_tcpip_t::callback_tod_status, this);
    pub_tod_status  = nh_data_.advertise<tod_msgs::Status>("Operator/Manager/tod_status", 10);

  } else {
    topic_tx_  = "/Operator/Control/control_cmd_data";
    sub_tx     = nh_data_.subscribe(topic_tx_, 10,
                  &driver_tcpip_t::tx_callback<tod_msgs::ControlCmd::ConstPtr>, this);
    pub_rx_payload_ = nh_data_.advertise<tod_msgs::ProbeVehicleData>("probe_vehicle_data", 10);

    std::string rx_front = "rx/front/encoded/h26x";
    std::string rx_left  = "rx/left/encoded/h26x";
    std::string rx_right = "rx/right/encoded/h26x";
    std::string rx_avm   = "rx/avm/encoded/h26x";
    pnh_.param<std::string>("rx_topic_video_front", rx_front, rx_front);
    pnh_.param<std::string>("rx_topic_video_left",  rx_left,  rx_left);
    pnh_.param<std::string>("rx_topic_video_right", rx_right, rx_right);
    pnh_.param<std::string>("rx_topic_video_avm",   rx_avm,   rx_avm);

    pubs_rx_video_.resize(kMaxStreams);
    // 수신 영상 publish는 video 쪽으로 묶어도 되고, data로 묶어도 되지만
    // 여기서는 영상 파이프라인이라는 의미로 nh_video_ 사용
    pubs_rx_video_[0] = nh_video_.advertise<h26x_encoder::EncodedFrame>(rx_front, 1);
    pubs_rx_video_[1] = nh_video_.advertise<h26x_encoder::EncodedFrame>(rx_left,  1);
    pubs_rx_video_[2] = nh_video_.advertise<h26x_encoder::EncodedFrame>(rx_right, 1);
    pubs_rx_video_[3] = nh_video_.advertise<h26x_encoder::EncodedFrame>(rx_avm,   1);

    sub_tod_status  = nh_data_.subscribe("Operator/Manager/tod_status", 5,
                                         &driver_tcpip_t::callback_tod_status, this);
    pub_tod_status  = nh_data_.advertise<tod_msgs::Status>("Vehicle/Manager/tod_status", 5);
  }

  pub_tcp_status  = nh_data_.advertise<tcpip_msgs::status>("v2x/tcp_status", 1);
  pub_v2x_stat_   = nh_data_.advertise<nr_v2x_msgs::V2XStat>("v2x/stats", 5);

  pub_tcp_rx_            = nh_data_.advertise<tcpip_msgs::tcp_packet>("v2x/rx_raw", 5);
  pub_modem_tx_status_   = nh_data_.advertise<nr_v2x_msgs::ModemTxStatus> ("v2x/status/modem_tx",   5);
  pub_modem_rx_status_   = nh_data_.advertise<nr_v2x_msgs::ModemRxStatus> ("v2x/status/modem_rx",   5);
  pub_comm_status_       = nh_data_.advertise<nr_v2x_msgs::CommUnitStatus>    ("v2x/status/comm",       5);
  pub_ctrl_status_       = nh_data_.advertise<nr_v2x_msgs::ControlUnitStatus> ("v2x/status/control",    5);
  pub_db_base_           = nh_data_.advertise<nr_v2x_msgs::V2XDataBase>       ("v2x/status/ssov",       5);

  status_timer_ = nh_data_.createTimer(ros::Duration(1.0),
                                  &driver_tcpip_t::on_status_timer, this);

  double ssov_period_ms = 1.0;
  pnh_.param<double>("ssov_period_ms", ssov_period_ms, ssov_period_ms);
  ssov_timer_ = nh_data_.createTimer(ros::Duration(ssov_period_ms/1000.0),
                                &driver_tcpip_t::on_send_ssov_timer, this);

  pnh_.param<int>("video_idle_reset_ms", video_idle_reset_ms_, video_idle_reset_ms_);

  // ---------- Services ----------
  srv_resolve_ip_       = nh_data_.advertiseService("resolve_ip",
                            &driver_tcpip_t::service_resolve_ip,       this);
  srv_start_tcp_server_ = nh_data_.advertiseService("start_tcp_server",
                            &driver_tcpip_t::service_start_tcp_server, this);
  srv_stop_tcp_server_  = nh_data_.advertiseService("stop_tcp_server",
                            &driver_tcpip_t::service_stop_tcp_server,  this);
  srv_start_tcp_client_ = nh_data_.advertiseService("start_tcp_client",
                            &driver_tcpip_t::service_start_tcp_client, this);
  srv_stop_tcp_client_  = nh_data_.advertiseService("stop_tcp_client",
                            &driver_tcpip_t::service_stop_tcp_client,  this);
  srv_open_udp_socket_  = nh_data_.advertiseService("open_udp_socket",
                            &driver_tcpip_t::service_open_udp_socket,  this);
  srv_close_socket_     = nh_data_.advertiseService("close_socket",
                            &driver_tcpip_t::service_close_socket,     this);
  srv_wsr_tx_           = nh_data_.advertiseService("wsr_tx",
                            &driver_tcpip_t::service_wsr_tx,           this);
  srv_v2x_config_       = nh_data_.advertiseService("v2x_config",
                            &driver_tcpip_t::service_v2x_config,       this);

  // ---------- ROS AsyncSpinner 시작 ----------
  // 영상 전용 스레드 1개, 데이터용 스레드 2개(필요하면 파라미터화 가능)
  video_spinner_.reset(new ros::AsyncSpinner(3, &video_queue_));
  data_spinner_.reset(new ros::AsyncSpinner(3, &data_queue_));
  video_spinner_->start();
  data_spinner_->start();

  // ---------- IO thread pool ----------
  int io_threads = std::max(2, static_cast<int>(std::thread::hardware_concurrency()/4));
  pnh_.param<int>("io_threads", io_threads, io_threads);
  io_threads = std::max(1, io_threads);
  io_threads_.reserve(io_threads);
  for (int i = 0; i < io_threads; ++i) {
    io_threads_.emplace_back([this](){
      try {
        m_io_context.run();
      } catch (const std::exception& e) {
        ROS_FATAL_STREAM("driver_tcpip_t io.run exception: " << e.what());
      }
    });
  }

  ROS_INFO_STREAM("driver_tcpip_t initialized"
                  << " video_frag_bytes=" << video_frag_bytes_
                  << " video_pacing_us="  << video_pacing_us_);
}

driver_tcpip_t::~driver_tcpip_t() {
  try {
    // ROS 스피너 먼저 정지
    if (video_spinner_) {
      video_spinner_->stop();
      video_spinner_.reset();
    }
    if (data_spinner_) {
      data_spinner_->stop();
      data_spinner_.reset();
    }

    if (build_pool_) {
      build_pool_->join();
      build_pool_.reset();
    }
    m_work_guard.reset();
    m_io_context.stop();
    for (auto& th : io_threads_) {
      if (th.joinable()) th.join();
    }
  } catch (...) {}
}

// ----------------------- ROS TX callbacks -----------------------
void driver_tcpip_t::callback_video0(const h26x_encoder::EncodedFrame::ConstPtr& msg) {
  if (!msg) return;
  const int      stream = msg->stream_id ? msg->stream_id : 0;
  const uint32_t fseq   = (msg->frame_id != 0) ? msg->frame_id : next_video_seq();
  auto buf = std::make_shared<std::vector<uint8_t>>(msg->data.begin(), msg->data.end());
  send_video_on_all(buf, stream, msg->keyframe, fseq);
}

void driver_tcpip_t::callback_video1(const h26x_encoder::EncodedFrame::ConstPtr& msg) {
  if (!msg) return;
  const int      stream = msg->stream_id ? msg->stream_id : 1;
  const uint32_t fseq   = (msg->frame_id != 0) ? msg->frame_id : next_video_seq();
  auto buf = std::make_shared<std::vector<uint8_t>>(msg->data.begin(), msg->data.end());
  send_video_on_all(buf, stream, msg->keyframe, fseq);
}

void driver_tcpip_t::callback_video2(const h26x_encoder::EncodedFrame::ConstPtr& msg) {
  if (!msg) return;
  const int      stream = msg->stream_id ? msg->stream_id : 2;
  const uint32_t fseq   = (msg->frame_id != 0) ? msg->frame_id : next_video_seq();
  auto buf = std::make_shared<std::vector<uint8_t>>(msg->data.begin(), msg->data.end());
  send_video_on_all(buf, stream, msg->keyframe, fseq);
}

void driver_tcpip_t::callback_video3(const h26x_encoder::EncodedFrame::ConstPtr& msg) {
  if (!msg) return;
  const int      stream = msg->stream_id ? msg->stream_id : 3;
  const uint32_t fseq   = (msg->frame_id != 0) ? msg->frame_id : next_video_seq();
  auto buf = std::make_shared<std::vector<uint8_t>>(msg->data.begin(), msg->data.end());
  send_video_on_all(buf, stream, msg->keyframe, fseq);
}

void driver_tcpip_t::callback_tod_status(const tod_msgs::Status::ConstPtr& msg) {
  if (!msg) return;
  try {
    namespace ser = ros::serialization;
    const size_t n = ser::serializationLength(*msg);
    if (!n) return;

    std::vector<uint8_t> buf(n);
    ros::serialization::OStream os(buf.data(), n);
    ser::serialize(os, *msg);

    auto sp = std::make_shared<std::vector<uint8_t>>(std::move(buf));
    std::atomic_store_explicit(&last_tod_status_, sp, std::memory_order_release);

    if (isVehicle_)
      last_tod_seq_.store(msg->vehicle_header.seq, std::memory_order_release);
    else
      last_tod_seq_.store(msg->operator_header.seq, std::memory_order_release);

  } catch (const std::exception& e) {
    ROS_WARN_STREAM("TOD status serialize failed: " << e.what());
  }
}

template <class MsgPtr>
void driver_tcpip_t::tx_callback(const MsgPtr& msg)
{
  if (!msg) return;
  try {
    namespace ser = ros::serialization;
    const size_t n = ser::serializationLength(*msg);
    if (!n) return;

    std::vector<uint8_t> buf(n);
    ros::serialization::OStream os(buf.data(), n);
    ser::serialize(os, *msg);

    auto sp = std::make_shared<std::vector<uint8_t>>(std::move(buf));
    std::atomic_store_explicit(&last_raw_payload_, sp, std::memory_order_release);

  } catch(const std::exception& e) {
    ROS_WARN_STREAM("RAW serialize failed: " << e.what());
  }
}

// ----------------------- SSOV 주기 송신 -----------------------
void driver_tcpip_t::on_send_ssov_timer(const ros::TimerEvent&) {
  const uint32_t cur  = last_tod_seq_.load(std::memory_order_acquire);
  const uint32_t sent = last_tod_seq_sent_.load(std::memory_order_acquire);
  if (cur == 0 || cur == sent) return;

  if (ssov_inflight_.test_and_set(std::memory_order_acq_rel)) return;

  // 스냅샷 로드
  auto tod_sp = std::atomic_load_explicit(&last_tod_status_,  std::memory_order_acquire);
  auto raw_sp = std::atomic_load_explicit(&last_raw_payload_, std::memory_order_acquire);

  if (!tod_sp || tod_sp->empty()) {
    ssov_inflight_.clear(std::memory_order_release);
    return;
  }

  boost::asio::post(m_io_context, [this, cur,
                                   tod_sp = std::move(tod_sp),
                                   raw_sp = std::move(raw_sp)]() {
    db_meta_.ulTimeStamp = sys_timer();

    std::vector<uint8_t> frame;
    if (overall_s_ == "v2") {
      v2x::proto::v2::BuildArgs a{};
      a.seq             = next_ssov_seq();
      a.psid            = psid_ext_;
      a.castMode        = cast_mode_;
      a.srcId           = src_id_;
      a.dstId           = dst_id_;
      a.overall_bitwize = 0x77;
      a.crc_variant     = v2x::crc16::Variant::XMODEM;
      a.outer_crc_mode  = v2x::proto::wire::OuterCrcMode::HostAdds;
      a.status.enabled  = false;

      v2x::proto::v2::add_ssov_item(a, db_meta_, tod_sp->data(), tod_sp->size(), /*own=*/false);
      if (raw_sp && !raw_sp->empty())
        v2x::proto::v2::add_tlvc_item(a, EM_PT_RAW_DATA, raw_sp->data(), raw_sp->size(), /*own=*/false);

      frame = v2x::proto::v2::build_tx_extensible(a);
    } else {
      v2x::proto::v1::BuildArgs a{};
      a.seq            = next_ssov_seq();
      a.psid           = psid_ext_;
      a.castMode       = cast_mode_;
      a.srcId          = src_id_;
      a.dstId          = dst_id_;
      a.crc_variant    = v2x::crc16::Variant::XMODEM;
      a.outer_crc_mode = v2x::proto::wire::OuterCrcMode::HostAdds;
      a.status.enabled = false;

      v2x::proto::v1::add_ssov_item(a, db_meta_, tod_sp->data(), tod_sp->size(), /*own=*/false);
      if (raw_sp && !raw_sp->empty())
        v2x::proto::v1::add_tlvc_item(a, EM_PT_RAW_DATA, raw_sp->data(), raw_sp->size(), /*own=*/false);

      frame = v2x::proto::v1::build_tx_extensible(a);
    }

    if (!frame.empty()) {
      for (auto& kv : m_tx_tcp_sockets) {
        auto& s = kv.second;
        if (s && s->is_open()) {
          m_link_metrics[kv.first].on_tx(ssov_seq16_.load(), frame.size());
          auto buf = std::make_shared<std::vector<uint8_t>>(frame); // copy 한 번
          s->enqueue_write(buf);
          // s->write(frame.data(), frame.size());
        }
      }
      last_tod_seq_sent_.store(cur, std::memory_order_release);
    }
    ssov_inflight_.clear(std::memory_order_release);
  });
}

// ----------------------- Build & Send (VIDEO RAW+MUX) -----------------------
void driver_tcpip_t::send_video_on_all(std::shared_ptr<std::vector<uint8_t>> frame,
                                       int stream_id, bool keyframe, uint32_t frame_seq)
{
  if (!frame || frame->empty()) return;

  auto build = [this](const uint8_t* mux, size_t mux_len,
                      uint16_t /*frag_idx*/, uint16_t /*frag_cnt*/,
                      uint16_t seq_for_header) -> std::vector<uint8_t>
  {
    return build_video_wire_(mux, mux_len, seq_for_header);
  };

  auto send  = [this](const std::vector<uint8_t>& wire, uint16_t seq_for_metrics) {
    send_wire_to_all_(wire, seq_for_metrics);
  };

  auto next_seq = [this]() -> uint16_t { return next_video_hdr_seq(); };

  auto vs = std::make_shared<VideoSender>(
      m_io_context, std::move(frame),
      stream_id, keyframe, frame_seq,
      video_frag_bytes_, video_pacing_us_,
      std::move(build), std::move(send), std::move(next_seq));
  vs->start();
}

std::vector<uint8_t> driver_tcpip_t::build_video_wire_(const uint8_t* mux, size_t mux_len, uint16_t seq) const
{
  if (!mux || mux_len == 0) return {};

  if (overall_s_ == "v2") {
    v2x::proto::v2::BuildArgs a{};
    a.seq            = seq;
    a.psid           = psid_ext_;
    a.castMode       = cast_mode_;
    a.srcId          = src_id_;
    a.dstId          = dst_id_;
    a.overall_bitwize= 0x77;
    a.crc_variant    = v2x::crc16::Variant::XMODEM;
    a.outer_crc_mode = v2x::proto::wire::OuterCrcMode::HostAdds;

    v2x::proto::v2::add_tlvc_item(a, EM_PT_VIDEO, mux, mux_len, /*own=*/false);
    a.status.enabled = false;
    return v2x::proto::v2::build_tx_extensible(a);
  } else {
    v2x::proto::v1::BuildArgs a{};
    a.seq            = seq;
    a.psid           = psid_ext_;
    a.castMode       = cast_mode_;
    a.srcId          = src_id_;
    a.dstId          = dst_id_;
    a.crc_variant    = v2x::crc16::Variant::XMODEM;
    a.outer_crc_mode = v2x::proto::wire::OuterCrcMode::HostAdds;

    v2x::proto::v1::add_tlvc_item(a, EM_PT_VIDEO, mux, mux_len, /*own=*/false);
    return v2x::proto::v1::build_tx_extensible(a);
  }
}

void driver_tcpip_t::send_wire_to_all_(const std::vector<uint8_t>& wire,
                                       uint16_t seq_for_metrics)
{
  for (auto& kv : m_tx_tcp_sockets) {
    auto& s = kv.second;
    if (!s || !s->is_open()) continue;
    m_link_metrics[kv.first].on_tx(seq_for_metrics, wire.size());
    auto buf = std::make_shared<std::vector<uint8_t>>(wire); // copy 한 번
    s->enqueue_write(buf);
    // s->write(wire.data(), wire.size());
  }
}

// ----------------------- TCP accept/connect -----------------------
void driver_tcpip_t::tcp_connection_sp(std::shared_ptr<boost::asio::ip::tcp::socket> sock) {
  ROS_INFO_STREAM("[tcp_connection_sp] invoked, sock.use_count="
                  << (sock ? sock.use_count() : 0));
  if (!sock) return;

  const uint32_t id = next_free_id(m_sockets);

  auto ts = std::make_shared<tcp_socket_t>(id, app_channel_t::UNKNOWN,
                                           m_io_context, std::move(sock));

  m_sockets[id]        = ts;
  m_tx_tcp_sockets[id] = ts;
  m_link_metrics.emplace(id, metrics::link_metrics_t{});

  ROS_INFO_STREAM("tcp socket#" << id << " added");
  publish_status();

  ts->start(
    // on_bytes
    [this, id](const uint8_t* d, std::size_t n){
      auto it = m_link_metrics.find(id);
      if (it != m_link_metrics.end()) it->second.on_rx(/*seq*/0, n);

      if (debug_rx_raw_) {
        tcpip_msgs::tcp_packet pkt;
        pkt.data.assign(d, d + n);
        try { pub_tcp_rx_.publish(pkt); }
        catch (const std::exception& e) { ROS_WARN_STREAM("pub_tcp_rx_ failed: " << e.what()); }
      }

      process_rx_bytes(id, d, n);
    },
    // on_closed
    [this, id](const boost::system::error_code& ec){
      ROS_WARN_STREAM("tcp socket#" << id << " closed: " << ec.message());
      m_sockets.erase(id);
      m_tx_tcp_sockets.erase(id);
      m_link_metrics.erase(id);
      publish_status();
    }
  );
}

// ----------------------- Services -----------------------
bool driver_tcpip_t::service_resolve_ip(tcpip_msgs::resolve_ip::Request& req,
                                        tcpip_msgs::resolve_ip::Response& res)
{
  udp::resolver resolver(m_io_context);
  boost::system::error_code ec;
  auto results = resolver.resolve(udp::v4(), req.hostname, "", ec);
  if (ec) {
    ROS_ERROR_STREAM("resolve_ip: resolver error for '" << req.hostname << "': " << ec.message());
    return false;
  }
  auto it = results.begin();
  if (it == results.end()) {
    ROS_ERROR_STREAM("resolve_ip: no results for '" << req.hostname << "'");
    return false;
  }
  res.ip = endpoint::to_ros(it->endpoint()).ip;
  ROS_INFO_STREAM("resolved '" << req.hostname << "' -> " << it->endpoint().address().to_string());
  return true;
}

bool driver_tcpip_t::service_start_tcp_server(tcpip_msgs::start_tcp_server::Request&  req,
                                              tcpip_msgs::start_tcp_server::Response& res)
{
  const auto id = next_free_id(m_tcp_servers);
  auto server = std::make_shared<tcp_server_t>(
      m_io_context, id,
      std::bind(&driver_tcpip_t::tcp_connection_sp, this, std::placeholders::_1)
  );

  if (server->start(req.local_endpoint)) {
    m_tcp_servers[id] = std::move(server);
    res.server_id = id;
    publish_status();
    ROS_INFO_STREAM("start_tcp_server: started id=" << id);
    return true;
  }
  ROS_ERROR_STREAM("start_tcp_server: failed id=" << id);
  return false;
}

bool driver_tcpip_t::service_stop_tcp_server(tcpip_msgs::stop_tcp_server::Request&  req,
                                             tcpip_msgs::stop_tcp_server::Response& /*res*/)
{
  auto it = m_tcp_servers.find(req.server_id);
  if (it == m_tcp_servers.end()) {
    ROS_ERROR_STREAM("stop_tcp_server: id=" << req.server_id << " not found");
    return false;
  }
  it->second->stop();
  m_tcp_servers.erase(it);
  publish_status();
  ROS_INFO_STREAM("stop_tcp_server: stopped id=" << req.server_id);
  return true;
}

bool driver_tcpip_t::service_start_tcp_client(tcpip_msgs::start_tcp_client::Request&  req,
                                              tcpip_msgs::start_tcp_client::Response& res)
{
  const auto id = next_free_id(m_tcp_clients);
  auto client = std::make_shared<tcp_client_t>(
      m_io_context, id,
      std::bind(&driver_tcpip_t::tcp_connection_sp, this, std::placeholders::_1)
  );
  if (client->start(req.local_endpoint, req.remote_endpoint)) {
    m_tcp_clients[id] = std::move(client);
    res.client_id = id;
    publish_status();
    ROS_INFO_STREAM("start_tcp_client: started id=" << id);
    return true;
  }
  ROS_ERROR_STREAM("start_tcp_client: failed id=" << id);
  return false;
}

bool driver_tcpip_t::service_stop_tcp_client(tcpip_msgs::stop_tcp_client::Request&  req,
                                             tcpip_msgs::stop_tcp_client::Response& /*res*/)
{
  auto it = m_tcp_clients.find(req.client_id);
  if (it == m_tcp_clients.end()) {
    ROS_ERROR_STREAM("stop_tcp_client: id=" << req.client_id << " not found");
    return false;
  }
  it->second->stop();
  m_tcp_clients.erase(it);
  publish_status();
  ROS_INFO_STREAM("stop_tcp_client: stopped id=" << req.client_id);
  return true;
}

bool driver_tcpip_t::service_open_udp_socket(tcpip_msgs::open_udp_socket::Request&  req,
                                             tcpip_msgs::open_udp_socket::Response& res)
{
  const auto id = next_free_id(m_sockets);
  auto sock = std::make_shared<udp_socket_t>(m_io_context, id);
  if (sock->open(req.local_endpoint)) {
    m_sockets[id] = sock;
    publish_status();
    res.socket_id = id;
    ROS_INFO_STREAM("open_udp_socket: opened id=" << id);
    return true;
  }
  ROS_ERROR_STREAM("open_udp_socket: failed");
  return false;
}

bool driver_tcpip_t::service_close_socket(tcpip_msgs::close_socket::Request&  req,
                                          tcpip_msgs::close_socket::Response& /*res*/)
{
  auto it = m_sockets.find(req.socket_id);
  if (it == m_sockets.end()) {
    ROS_ERROR_STREAM("close_socket: id=" << req.socket_id << " not found");
    return false;
  }
  it->second->close();
  m_sockets.erase(it);
  m_tx_tcp_sockets.erase(req.socket_id);
  m_link_metrics.erase(req.socket_id);
  publish_status();
  ROS_INFO_STREAM("close_socket: closed id=" << req.socket_id);
  return true;
}

bool driver_tcpip_t::service_v2x_config(nr_v2x_msgs::V2XConfig::Request&  req,
                                        nr_v2x_msgs::V2XConfig::Response& res)
{
  psid_ext_  = req.psid_ext;
  cast_mode_ = req.cast_mode;
  src_id_    = req.src_id;
  dst_id_    = req.dst_id;
  db_meta_.eDeviceType   = req.eDeviceType;
  db_meta_.eTeleCommType = req.eTeleCommType;
  db_meta_.unDeviceId    = req.unDeviceId;
  db_meta_.eServiceId    = req.eServiceId;
  db_meta_.eActionType   = req.eActionType;
  db_meta_.eRegionId     = req.eRegionId;
  db_meta_.ePayloadType  = req.ePayloadType;
  db_meta_.eCommId       = req.eCommId;
  db_meta_.usDbVer       = req.usDbVer;
  db_meta_.usHwVer       = req.usHwVer;
  db_meta_.usSwVer       = req.usSwVer;

  res.success       = true;
  res.message       = "ok";

  return true;
}

// ----------------------- WSR Service -----------------------
bool driver_tcpip_t::service_wsr_tx(nr_v2x_msgs::Wsr::Request& req,
                                    nr_v2x_msgs::Wsr::Response& res) {
  const uint16_t seq = next_wsr_seq();
  std::vector<uint8_t> frame = v2x::proto::wire::build_wsr_frame(
      seq, req.action, req.request_psid, crc_variant_,
      v2x::proto::wire::OuterCrcMode::HostAdds);
  ROS_INFO_STREAM("[WSR-TX] seq=" << seq
        << " action=" << (int)req.action
        << " psid="   << req.request_psid
        << " bytes="  << frame.size());
  ROS_INFO_STREAM("[WSR-TX-HEX]\n" << hex_dump(frame.data(), frame.size(), 160));
  std::size_t sent_count = 0;
  for (auto& kv : m_tx_tcp_sockets) {
    auto& s = kv.second;
    if (!s || !s->is_open()) continue;
    try {
      s->write(frame.data(), frame.size());
      ++sent_count;
    } catch (const std::exception& e) {
      ROS_WARN_STREAM("wsr send failed on socket#" << kv.first << ": " << e.what());
    }
  }
  if (sent_count == 0) {
    res.success = false;
    res.message = "no open TCP sockets to send";
    return true;
  }

  // 응답 대기 (최대 10s)
  WscReply reply{};
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  {
    std::unique_lock<std::mutex> lk(wsr_mtx_);
    bool got = wsr_cv_.wait_until(lk, deadline,
                 [&]{ return wsc_inbox_.find(seq) != wsc_inbox_.end(); });
    if (!got) {
      res.success = false;
      res.message = "WSC timeout";
      return true;
    }
    reply = wsc_inbox_[seq];
    wsc_inbox_.erase(seq);
  }

  res.success       = true;
  res.message       = "ok";
  res.action_result = reply.action_result;
  res.response_psid = reply.psid;

  ROS_INFO_STREAM("WSR done: seq=" << seq
                  << " action=" << (int)req.action
                  << " psid="   << req.request_psid
                  << " -> result=" << (int)reply.action_result
                  << " resp_psid=" << reply.psid);
  return true;
}

// ----------------------- status publish -----------------------
void driver_tcpip_t::publish_status() const {
  tcpip_msgs::status msg;

  for (const auto& kv : m_tcp_servers) msg.tcp_servers.push_back(kv.second->description());
  for (const auto& kv : m_tcp_clients) msg.tcp_clients.push_back(kv.second->description());

  for (const auto& kv : m_sockets) {
    switch (kv.second->protocol()) {
      case protocol_t::TCP:
        if (auto* tcp_sock = dynamic_cast<tcp_socket_t*>(kv.second.get()))
          msg.tcp_sockets.push_back(tcp_sock->description());
        break;
      case protocol_t::UDP:
        if (auto* udp_sock = dynamic_cast<udp_socket_t*>(kv.second.get()))
          msg.udp_sockets.push_back(udp_sock->description());
        break;
    }
  }

  for (const auto& kv : m_sockets) {
    const uint32_t sid = kv.first;
    auto it = m_link_metrics.find(sid);
    if (it == m_link_metrics.end()) continue;

    const auto snap = it->second.sample_and_reset_window();

    tcpip_msgs::LinkMetrics lm;
    lm.socket_id        = sid;
    lm.label            = kv.second->label();
    lm.channel          = static_cast<uint8_t>(kv.second->channel());
    lm.stamp            = ros::Time::now();

    lm.tx_bps           = snap.tx_bps;
    lm.rx_bps           = snap.rx_bps;
    lm.tx_frames_delta  = snap.tx_frames_delta;
    lm.rx_frames_delta  = snap.rx_frames_delta;
    lm.rx_missing_delta = snap.rx_missing_delta;
    lm.rx_dup_delta     = snap.rx_dup_delta;
    lm.rx_reorder_delta = snap.rx_reorder_delta;
    lm.pdr_rx           = snap.pdr_rx;

    lm.tx_frames        = snap.tx_frames;
    lm.rx_frames        = snap.rx_frames;
    lm.rx_missing       = snap.rx_missing;
    lm.rx_dup           = snap.rx_dup;
    lm.rx_reorder       = snap.rx_reorder;
    lm.tx_bytes         = snap.tx_bytes;
    lm.rx_bytes         = snap.rx_bytes;

    msg.socket_metrics.push_back(lm);
  }

  pub_tcp_status.publish(msg);
}

void driver_tcpip_t::on_status_timer(const ros::TimerEvent&) {
  publish_status();
  publish_v2x_stat();
}

void driver_tcpip_t::publish_v2x_stat() {

  const uint16_t modem_us = static_cast<uint16_t>(last_rx_devts_ - last_tx_devts_);
  
  uint8_t ssov_pdr = 0;
  {
    std::lock_guard<std::mutex> lk(ssov_pdr_mtx_);
    ssov_pdr = ssov_pdr_.pdr_window_and_reset();
  }

  std::array<uint8_t, kMaxStreams> video_pdr_val{};
  const ros::Time now = ros::Time::now();

  for (int s = 0; s < kMaxStreams; ++s) {
    std::lock_guard<std::mutex> lk(video_pdr_mtx_[s]);
    if (video_idle_reset_ms_ > 0) {
      const bool idle =
        video_last_seen_[s].isZero() ||
        (now - video_last_seen_[s]).toSec() * 1000.0 > video_idle_reset_ms_;
      if (idle) {
        video_pdr_[s].seen = false;
        video_pdr_[s].last_seq = 0;
        video_pdr_[s].rx_win = video_pdr_[s].miss_win = 0;
      }
    }
    video_pdr_val[s] = video_pdr_[s].pdr_window_and_reset();
  }

  uint16_t dist_m = 0;
  if ((last_tx_lat_ | last_tx_lon_ | last_rx_lat_ | last_rx_lon_) != 0) {
    const double tx_lat = last_tx_lat_ / 1'000'000.0, tx_lon = last_tx_lon_ / 1'000'000.0;
    const double rx_lat = last_rx_lat_ / 1'000'000.0, rx_lon = last_rx_lon_ / 1'000'000.0;
    const double d = haversine_m(tx_lat, tx_lon, rx_lat, rx_lon);
    dist_m = static_cast<uint16_t>(std::min<double>(std::max<double>(0, d), 65535.0));
  }

  nr_v2x_msgs::V2XStat stat;
  stat.header.stamp  = ros::Time::now();
  stat.e2e_latency   = clamp_ms16_from_us(e2e_latency_us_accum_);
  stat.modem_latency = modem_us;
  stat.distance      = dist_m;
  stat.ssov_pdr      = ssov_pdr;
  stat.video_pdr.assign(video_pdr_val.begin(), video_pdr_val.end());
  pub_v2x_stat_.publish(stat);
}

// ----------------------- RX path -----------------------
void driver_tcpip_t::process_rx_bytes(uint32_t sid, const uint8_t* data, std::size_t n)
{
  auto& buf = rx_bufs_[sid];
  const std::size_t prev = buf.size();
  buf.resize(prev + n);
  std::memcpy(buf.data() + prev, data, n);

  while (true) {
    std::size_t want = 0;
    if (!v2x::proto::wire::peek_total_len_no_tail(buf.data(), buf.size(), want))
      break;
    if (buf.size() < want) break;

    v2x::proto::wire::PacketHeader hdr{};
    if (!v2x::proto::wire::parse_and_verify_header(
            buf.data(), want, hdr,
            v2x::proto::wire::TailCrcPolicy::Skip))
    {
      // 재동기화: 1바이트 drop
      std::vector<uint8_t> rest(buf.begin() + 1, buf.end());
      buf.swap(rest);
      continue;
    }

    bool ok = false;

    switch (hdr.payload_id) {
      case ePayloadId_WsmServiceConfirm: {
        constexpr std::size_t hdr_size = 4 + 2 + 2 + 2; // 10
        if (want < hdr_size) { ok = false; break; }

        const uint8_t* payload = buf.data() + hdr_size;
        const std::size_t plen = want - hdr_size;

        const uint16_t seq = hdr.seq;
        uint8_t  action_result = 0;
        uint32_t psid = 0;

        if (v2x::proto::handle_wsc_payload(payload, plen, crc_variant_, action_result, psid))
        {
          {
            std::lock_guard<std::mutex> lk(wsr_mtx_);
            wsc_inbox_[seq] = WscReply{seq, action_result, psid};
          }
          wsr_cv_.notify_all();
          ok = true;
        } else {
          ok = false;
        }
        break;
      }

      case ePayloadId_RxMsg: {
        constexpr std::size_t hdr_size = 4 + 2 + 2 + 2;
        if (want < hdr_size) { ok = false; break; }

        const uint8_t* payload = buf.data() + hdr_size;
        const std::size_t plen = want - hdr_size;

        v2x::proto::RxHandlers handlers;
        handlers.on_video = std::bind(&driver_tcpip_t::on_video_tlvc, this,
                                      std::placeholders::_1);
        handlers.on_raw   = std::bind(&driver_tcpip_t::on_raw, this,
                                      std::placeholders::_1,
                                      std::placeholders::_2,
                                      std::placeholders::_3);
        handlers.on_status_tx_modem_v2 = std::bind(&driver_tcpip_t::on_status_tx_modem_v2, this, std::placeholders::_1);
        handlers.on_status_rx_modem_v2 = std::bind(&driver_tcpip_t::on_status_rx_modem_v2, this, std::placeholders::_1);
        handlers.on_status_comm_unit_v2= std::bind(&driver_tcpip_t::on_status_comm_unit_v2, this, std::placeholders::_1);
        handlers.on_status_ctrl_unit_v2= std::bind(&driver_tcpip_t::on_status_ctrl_unit_v2, this, std::placeholders::_1);
        handlers.on_status_tx_modem_v1 = std::bind(&driver_tcpip_t::on_status_tx_modem_v1, this, std::placeholders::_1);
        handlers.on_status_rx_modem_v1 = std::bind(&driver_tcpip_t::on_status_rx_modem_v1, this, std::placeholders::_1);
        handlers.on_status_comm_unit_v1= std::bind(&driver_tcpip_t::on_status_comm_unit_v1, this, std::placeholders::_1);
        handlers.on_status_ctrl_unit_v1= std::bind(&driver_tcpip_t::on_status_ctrl_unit_v1, this, std::placeholders::_1);
        handlers.on_ssov = std::bind(&driver_tcpip_t::on_ssov, this,
                                     std::placeholders::_1,
                                     std::placeholders::_2,
                                     std::placeholders::_3);

        v2x::proto::RxOptions opts;
        opts.status_only_when_ssov = true;
        opts.strict_crc            = false;
        opts.vehicle_mode          = isVehicle_;

        ok = v2x::proto::dispatch_rxmsg(payload, plen, handlers, opts);
        break;
      }

      default:
        ok = false;
        break;
    }

    if (!ok) {
      std::vector<uint8_t> rest(buf.begin() + 1, buf.end());
      buf.swap(rest);
      continue;
    }

    if (buf.size() == want) { buf.clear(); break; }
    std::vector<uint8_t> rest(buf.begin() + want, buf.end());
    buf.swap(rest);
  }

  if (buf.size() > 512 * 1024) {
    ROS_WARN_STREAM("rx buffer cap exceeded on socket#" << sid << ", dropping");
    buf.clear();
  }
}

// ----------------------- util -----------------------
uint64_t driver_tcpip_t::sys_timer() noexcept {
  using namespace std::chrono;
  return duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
}

uint16_t driver_tcpip_t::clamp_ms16_from_us(uint64_t us) {
  uint64_t ms = us / 1000ULL;
  if (ms > 0xFFFFu) ms = 0xFFFFu;
  return static_cast<uint16_t>(ms);
}

double driver_tcpip_t::haversine_m(double lat1, double lon1, double lat2, double lon2) {
  constexpr double R = 6371000.0;
  auto rad = [](double d){ return d * M_PI / 180.0; };
  double dlat = rad(lat2 - lat1), dlon = rad(lon2 - lon1);
  double a = std::sin(dlat/2)*std::sin(dlat/2) +
             std::cos(rad(lat1))*std::cos(rad(lat2))*std::sin(dlon/2)*std::sin(dlon/2);
  return R * 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));
}

// ----------------------- RX: video TLVC → 즉시 publish -----------------------
void driver_tcpip_t::on_video_tlvc(const v2x::proto::wire::TlvcView& tv) {
  mux_bank_.feed_bytes(tv.v, tv.v_len,
    [this](uint8_t stream, bool key, uint32_t fseq, const uint8_t* p, size_t n){
      if (!p || n == 0) return;
      if (stream >= kMaxStreams) return;
      if (stream >= pubs_rx_video_.size()) return;
      if (!pubs_rx_video_[stream]) return;

      // PDR 집계 (락은 최대한 짧게)
      if (video_pdr_mtx_[stream].try_lock()) {
        auto& ctr = video_pdr_[stream];
        ctr.maybe_handle_reset(fseq);
        ctr.on_seq(fseq);
        video_last_seen_[stream] = ros::Time::now();
        video_pdr_mtx_[stream].unlock();
      }

      h26x_encoder::EncodedFrame msg;
      msg.header.stamp = ros::Time::now();  // 수신 시각 기준
      msg.keyframe     = key;
      msg.stream_id    = stream;
      msg.frame_id     = fseq;
      msg.data.assign(p, p + n);

      try {
        pubs_rx_video_[stream].publish(msg);
      } catch (const std::exception& ex) {
        ROS_ERROR_STREAM("publish video failed: " << ex.what());
      }
    });
}

// ----------------------- 상태 콜백들 -----------------------
void driver_tcpip_t::on_status_tx_modem_v2(const TLVC_STATUS_Tx_ModemUnit_V2* s){
  last_tx_devts_ = v2x::be64_from(&s->timestamp);
  last_tx_lat_   = static_cast<int32_t>(v2x::be32_from(&s->latitude));
  last_tx_lon_   = static_cast<int32_t>(v2x::be32_from(&s->longitude));

  nr_v2x_msgs::ModemTxStatus m;
  m.header.stamp     = ros::Time::now();
  m.dev_type         = s->dev_type;
  m.tx_rx            = s->tx_rx;
  m.dev_id           = v2x::be32_from(&s->dev_id);
  m.hw_ver           = v2x::be16_from(&s->hw_ver);
  m.sw_ver           = v2x::be16_from(&s->sw_ver);
  m.device_timestamp = v2x::be64_from(&s->timestamp);
  m.tx_power         = static_cast<int8_t>(s->tx_power);
  m.freq             = v2x::be16_from(&s->freq);
  m.bandwidth        = s->bandwidth;
  m.scs              = s->scs;
  m.mcs              = s->mcs;
  m.latitude         = static_cast<int32_t>(v2x::be32_from(&s->latitude));
  m.longitude        = static_cast<int32_t>(v2x::be32_from(&s->longitude));
  m.cpu_temp         = static_cast<int8_t>(s->cpu_temp);
  m.peri_temp        = static_cast<int8_t>(s->peri_temp);
  pub_modem_tx_status_.publish(m);
}

void driver_tcpip_t::on_status_rx_modem_v2(const TLVC_STATUS_Rx_ModemUnit_V2* s){
  last_rx_devts_ = v2x::be64_from(&s->timestamp);
  last_rx_lat_   = static_cast<int32_t>(v2x::be32_from(&s->latitude));
  last_rx_lon_   = static_cast<int32_t>(v2x::be32_from(&s->longitude));

  nr_v2x_msgs::ModemRxStatus m;
  m.header.stamp     = ros::Time::now();
  m.dev_type         = s->dev_type;
  m.tx_rx            = s->tx_rx;
  m.dev_id           = v2x::be32_from(&s->dev_id);
  m.hw_ver           = v2x::be16_from(&s->hw_ver);
  m.sw_ver           = v2x::be16_from(&s->sw_ver);
  m.device_timestamp = v2x::be64_from(&s->timestamp);
  m.rssi             = static_cast<int8_t>(s->rssi);
  m.rcpi             = s->rcpi;
  m.latitude         = static_cast<int32_t>(v2x::be32_from(&s->latitude));
  m.longitude        = static_cast<int32_t>(v2x::be32_from(&s->longitude));
  m.cpu_temp         = static_cast<int8_t>(s->cpu_temp);
  m.peri_temp        = static_cast<int8_t>(s->peri_temp);
  pub_modem_rx_status_.publish(m);
}

void driver_tcpip_t::on_status_comm_unit_v2(const TLVC_STATUS_CommUnit_V2* s){
  nr_v2x_msgs::CommUnitStatus m;
  m.header.stamp     = ros::Time::now();
  m.dev_type         = s->dev_type;
  m.tx_rx            = s->tx_rx;
  m.dev_id           = v2x::be32_from(&s->dev_id);
  m.hw_ver           = v2x::be16_from(&s->hw_ver);
  m.sw_ver           = v2x::be16_from(&s->sw_ver);
  m.device_timestamp = v2x::be64_from(&s->timestamp);
  m.cpu_temp         = static_cast<int8_t>(s->cpu_temp);
  m.peri_temp        = static_cast<int8_t>(s->peri_temp);
  pub_comm_status_.publish(m);
}

void driver_tcpip_t::on_status_ctrl_unit_v2(const TLVC_STATUS_ControlUnit_V2* s){
  nr_v2x_msgs::ControlUnitStatus m;
  m.header.stamp     = ros::Time::now();
  m.dev_type         = s->dev_type;
  m.tx_rx            = s->tx_rx;
  m.dev_id           = v2x::be32_from(&s->dev_id);
  m.hw_ver           = v2x::be16_from(&s->hw_ver);
  m.sw_ver           = v2x::be16_from(&s->sw_ver);
  m.device_timestamp = v2x::be64_from(&s->timestamp);
  m.cpu_temp         = static_cast<int8_t>(s->cpu_temp);
  m.peri_temp        = static_cast<int8_t>(s->peri_temp);
  pub_ctrl_status_.publish(m);
}

// V1 상태들
void driver_tcpip_t::on_status_tx_modem_v1(const TLVC_STATUS_Tx_ModemUnit* s){
  last_tx_devts_ = v2x::be64_from(&s->timestamp);
  last_tx_lat_   = static_cast<int32_t>(v2x::be32_from(&s->latitude));
  last_tx_lon_   = static_cast<int32_t>(v2x::be32_from(&s->longitude));
}
void driver_tcpip_t::on_status_rx_modem_v1(const TLVC_STATUS_Rx_ModemUnit* s){
  last_rx_devts_ = v2x::be64_from(&s->timestamp);
  last_rx_lat_   = static_cast<int32_t>(v2x::be32_from(&s->latitude));
  last_rx_lon_   = static_cast<int32_t>(v2x::be32_from(&s->longitude));
}
void driver_tcpip_t::on_status_comm_unit_v1(const TLVC_STATUS_CommUnit*){}
void driver_tcpip_t::on_status_ctrl_unit_v1(const TLVC_STATUS_ControlUnit*){}

// RAW TLVC → ROS msg
void driver_tcpip_t::on_raw(uint32_t tlvc_type, const uint8_t* p, size_t n){
  if(tlvc_type != EM_PT_RAW_DATA) return;

  if (isVehicle_) {
    tod_msgs::ControlCmd cmd;
    if (deserialize_ros_msg(p, n, cmd)) {
      pub_rx_payload_.publish(cmd);
    } else {
      ROS_WARN("RAW payload deserialize to ControlCmd failed");
    }
  } else {
    tod_msgs::ProbeVehicleData pvd;
    if (deserialize_ros_msg(p, n, pvd)) {
      pub_rx_payload_.publish(pvd);
    } else {
      ROS_WARN("RAW payload deserialize to ProbeVehicleData failed");
    }
  }
}

// SSOV 처리
void driver_tcpip_t::on_ssov(const v2x::proto::common::DbV2xFixed& meta,
                             const uint8_t* p, size_t n)
{
  const uint64_t now_us = sys_timer();
  const uint64_t src_us = meta.ulTimeStamp;
  const uint64_t diff_us = (now_us > src_us) ? (now_us - src_us) : 0ULL;

  e2e_latency_us_accum_ = diff_us;

  nr_v2x_msgs::V2XDataBase m;
  m.eDeviceType     = meta.eDeviceType;
  m.eTeleCommType   = meta.eTeleCommType;
  m.unDeviceId      = meta.unDeviceId;
  m.ulTimeStamp     = meta.ulTimeStamp;
  m.eServiceId      = meta.eServiceId;
  m.eActionType     = meta.eActionType;
  m.eRegionId       = meta.eRegionId;
  m.ePayloadType    = meta.ePayloadType;
  m.eCommId         = meta.eCommId;
  m.usDbVer         = meta.usDbVer;
  m.usHwVer         = meta.usHwVer;
  m.usSwVer         = meta.usSwVer;
  m.ulPayloadLength = meta.ulPayloadLength;
  pub_db_base_.publish(m);

  tod_msgs::Status status;
  if (deserialize_ros_msg(p, n, status)) {
    {
      std::lock_guard<std::mutex> lk(ssov_pdr_mtx_);
      if (isVehicle_) {
        ssov_pdr_.maybe_handle_reset(status.operator_header.seq);
        ssov_pdr_.on_seq(status.operator_header.seq);
      } else {
        ssov_pdr_.maybe_handle_reset(status.vehicle_header.seq);
        ssov_pdr_.on_seq(status.vehicle_header.seq);
      }
    }
    pub_tod_status.publish(status);
  } else {
    ROS_WARN("DBV2X payload deserialize to ToD Status failed");
  }
}

std::string driver_tcpip_t::hex_dump(const uint8_t* p, size_t n, size_t maxn) {
  std::ostringstream oss;
  size_t len = std::min(n, maxn);
  for (size_t i=0;i<len;++i) {
    if (i % 16 == 0) oss << std::setw(4) << std::setfill('0') << std::hex << i << ": ";
    oss << std::setw(2) << std::setfill('0') << std::uppercase << std::hex << (int)p[i] << ' ';
    if (i % 16 == 15) oss << '\n';
  }
  if (n > maxn) oss << "... (" << (n-maxn) << " more bytes)\n";
  return oss.str();
}

template <typename MsgT>
bool driver_tcpip_t::deserialize_ros_msg(const uint8_t* ptr, std::size_t len, MsgT& out) {
  if (!ptr || len == 0) return false;
  try {
    ros::serialization::IStream is(const_cast<uint8_t*>(ptr), len);
    ros::serialization::deserialize(is, out);
    return true;
  } catch (...) {
    return false;
  }
}

} // namespace v2x_interface
