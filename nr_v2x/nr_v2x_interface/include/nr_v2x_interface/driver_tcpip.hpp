#pragma once
/// \file driver_tcpip.hpp
/// \brief V2X TCP driver with ROS services (resolve/start/stop/open/close) + TLVC sending/receiving.

#include <ros/ros.h>
#include <ros/serialization.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>

#include <tod_msgs/ProbeVehicleData.h>
#include <tod_msgs/ControlCmd.h>
#include <tod_msgs/Status.h>
#include <h26x_encoder/EncodedFrame.h>

#include <tcpip_msgs/status.h>
#include <tcpip_msgs/endpoint.h>
#include <tcpip_msgs/tcp_packet.h>

// services
#include <tcpip_msgs/resolve_ip.h>
#include <tcpip_msgs/start_tcp_server.h>
#include <tcpip_msgs/stop_tcp_server.h>
#include <tcpip_msgs/start_tcp_client.h>
#include <tcpip_msgs/stop_tcp_client.h>
#include <tcpip_msgs/open_udp_socket.h>
#include <tcpip_msgs/close_socket.h>

#include <nr_v2x_msgs/ModemTxStatus.h>
#include <nr_v2x_msgs/ModemRxStatus.h>
#include <nr_v2x_msgs/CommUnitStatus.h>
#include <nr_v2x_msgs/ControlUnitStatus.h>
#include <nr_v2x_msgs/V2XDataBase.h>
#include <nr_v2x_msgs/Wsr.h>
#include <nr_v2x_msgs/V2XConfig.h>
#include <nr_v2x_msgs/V2XStat.h>   // latency / PDR / distance 통계

#include <boost/asio/io_context.hpp>
#include <boost/asio/executor_work_guard.hpp>
#include <boost/asio/ip/udp.hpp>
#include <boost/asio/thread_pool.hpp>

#include <thread>
#include <map>
#include <memory>
#include <atomic>
#include <unordered_map>
#include <mutex>
#include <condition_variable>
#include <array>
#include <deque>
#include <vector>
#include <cmath>

// protocol / sockets / metrics
#include "nr_v2x_interface/protocol.hpp"
#include "nr_v2x_interface/endpoint.hpp"
#include "nr_v2x_interface/tcp_client.hpp"
#include "nr_v2x_interface/tcp_server.hpp"
#include "nr_v2x_interface/tcp_socket.hpp"
#include "nr_v2x_interface/udp_socket.hpp"
#include "nr_v2x_interface/socket.hpp"
#include "nr_v2x_interface/protocol/tx_builder.hpp"
#include "nr_v2x_interface/protocol/rx_parser.hpp"
#include "nr_v2x_interface/metrics/link_metrics.hpp"
#include <nr_v2x_helper/crc16.hpp>
#include "nr_v2x_interface/video_sender.hpp"

namespace v2x_interface {

// ---------------- PDR Counter ----------------
struct PdrCounter {
  uint32_t last_seq{0};
  bool     seen{false};
  uint32_t rx{0}, miss{0};          // 전체 누적
  uint32_t rx_win{0}, miss_win{0};  // 윈도우(1Hz 등) 누적

  void maybe_handle_reset(uint32_t seq) {
    if (!seen) return;
    if (seq <= last_seq) {
      uint32_t back = last_seq - seq;
      // wrap-around 또는 큰 점프 → 리셋
      if (back > 1000) {
        seen = false;
        last_seq = 0;
        rx_win = miss_win = 0;
      }
    }
  }

  void on_seq(uint32_t seq) {
    if (!seen) {
      seen    = true;
      last_seq = seq;
      rx++;
      rx_win++;
      return;
    }
    uint32_t gap = seq - last_seq;  // unsigned wrap-around 포함
    if (gap > 0) {
      if (gap > 1) {
        miss     += (gap - 1);
        miss_win += (gap - 1);
      }
      rx++;
      rx_win++;
      last_seq = seq;
    } else {
      // duplicate인 경우: 필요하면 따로 카운트 가능
    }
  }

  uint8_t pdr_window_and_reset() {
    uint8_t  p   = 0;
    uint32_t tot = rx_win + miss_win;
    if (tot) {
      p = static_cast<uint8_t>(std::min<uint32_t>(100, (rx_win * 100U) / tot));
    }
    rx_win = miss_win = 0;
    return p;
  }

  uint8_t pdr_total() const {
    uint32_t tot = rx + miss;
    if (!tot) return 0;
    return static_cast<uint8_t>(std::min<uint32_t>(100, (rx * 100U) / tot));
  }
};

// ---------------- driver_tcpip_t ----------------
class driver_tcpip_t {
public:
  explicit driver_tcpip_t(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~driver_tcpip_t();

  /// H.26x 압축 프레임(1개)을 모든 TCP 세션으로 전송 (조각화 + TLVC 래핑)
  void send_video_on_all(std::shared_ptr<std::vector<uint8_t>> frame,
                         int stream_id, bool keyframe, uint32_t frame_seq);

private:
  // ---------- ROS Services ----------
  bool service_resolve_ip       (tcpip_msgs::resolve_ip::Request&  req,
                                 tcpip_msgs::resolve_ip::Response& res);
  bool service_start_tcp_server (tcpip_msgs::start_tcp_server::Request&  req,
                                 tcpip_msgs::start_tcp_server::Response& res);
  bool service_stop_tcp_server  (tcpip_msgs::stop_tcp_server::Request&   req,
                                 tcpip_msgs::stop_tcp_server::Response&  res);
  bool service_start_tcp_client (tcpip_msgs::start_tcp_client::Request&  req,
                                 tcpip_msgs::start_tcp_client::Response& res);
  bool service_stop_tcp_client  (tcpip_msgs::stop_tcp_client::Request&   req,
                                 tcpip_msgs::stop_tcp_client::Response&  res);
  bool service_open_udp_socket  (tcpip_msgs::open_udp_socket::Request&   req,
                                 tcpip_msgs::open_udp_socket::Response&  res);
  bool service_close_socket     (tcpip_msgs::close_socket::Request&      req,
                                 tcpip_msgs::close_socket::Response&     res);
  bool service_wsr_tx           (nr_v2x_msgs::Wsr::Request& req,
                                 nr_v2x_msgs::Wsr::Response& res);
  bool service_v2x_config       (nr_v2x_msgs::V2XConfig::Request&  req,
                                 nr_v2x_msgs::V2XConfig::Response& res);

  // ---------- ROS Data IO ----------
  void callback_video0(const h26x_encoder::EncodedFrame::ConstPtr& msg);
  void callback_video1(const h26x_encoder::EncodedFrame::ConstPtr& msg);
  void callback_video2(const h26x_encoder::EncodedFrame::ConstPtr& msg);
  void callback_video3(const h26x_encoder::EncodedFrame::ConstPtr& msg);

  void callback_tod_status(const tod_msgs::Status::ConstPtr& msg);

  void send_wire_to_all_(const std::vector<uint8_t>& wire,
                         uint16_t seq_for_metrics);

  template <class MsgPtr>
  void tx_callback(const MsgPtr& msg);

  void on_status_timer(const ros::TimerEvent&);
  void publish_status() const;
  void publish_v2x_stat();

  void on_send_ssov_timer(const ros::TimerEvent&);

  // ---------- wire helpers ----------
  void send_ext_raw_on_all (const uint8_t* data, std::size_t n, bool keyframe);
  void send_ext_ssov_on_all(const uint8_t* data, std::size_t n);

  // ---------- connection events ----------
  void tcp_connection_sp(std::shared_ptr<boost::asio::ip::tcp::socket> sock);

  // ---------- RX path ----------
  void process_rx_bytes(uint32_t sid, const uint8_t* data, std::size_t n);

  // ---------- time/geo utils ----------
  uint64_t sys_timer() noexcept;
  uint16_t clamp_ms16_from_us(uint64_t us);
  static double haversine_m(double lat1, double lon1,
                            double lat2, double lon2);

  // ---------- rx_parser 콜백 ----------
  // 비디오 TLVC → raw-mux 재조립 → 즉시 EncodedFrame publish
  void on_video_tlvc(const v2x::proto::wire::TlvcView& tv);

  void on_status_tx_modem_v2(const TLVC_STATUS_Tx_ModemUnit_V2* s);
  void on_status_rx_modem_v2(const TLVC_STATUS_Rx_ModemUnit_V2* s);
  void on_status_comm_unit_v2(const TLVC_STATUS_CommUnit_V2* s);
  void on_status_ctrl_unit_v2(const TLVC_STATUS_ControlUnit_V2* s);

  void on_status_tx_modem_v1(const TLVC_STATUS_Tx_ModemUnit* s);
  void on_status_rx_modem_v1(const TLVC_STATUS_Rx_ModemUnit* s);
  void on_status_comm_unit_v1(const TLVC_STATUS_CommUnit* s);
  void on_status_ctrl_unit_v1(const TLVC_STATUS_ControlUnit* s);

  // SSOV (outer seq 포함)
  void on_ssov(const v2x::proto::common::DbV2xFixed& meta,
               const uint8_t* p, size_t n);
  void on_raw (uint32_t tlvc_type,
               const uint8_t* p, size_t n);

  template <typename MsgT>
  bool deserialize_ros_msg(const uint8_t* ptr, std::size_t len, MsgT& out);

  // ---------- socket book-keeping ----------
  template <class MapT>
  static uint32_t next_free_id(const MapT& m) {
    uint32_t id = 1;
    while (m.find(id) != m.end()) ++id;
    return id;
  }

  static std::string hex_dump(const uint8_t* p,
                              size_t n,
                              size_t maxn = 256);

  // ---------- seq ----------
  uint16_t next_ssov_seq() noexcept      { return ++ssov_seq16_; }
  uint16_t next_video_hdr_seq() noexcept { return ++video_hdr_seq16_; }
  uint16_t next_raw_seq() noexcept       { return ++raw_seq16_; }
  uint16_t next_wsr_seq() noexcept       { return ++wsr_seq16_; }
  uint32_t next_video_seq() noexcept {
    return video_seq_.fetch_add(1, std::memory_order_relaxed);
  }

private:
  // ==== ROS NodeHandles & CallbackQueues ====
  ros::NodeHandle nh_;    ///< 기본 NodeHandle
  ros::NodeHandle pnh_;   ///< private NodeHandle (~driver_tcpip)
  // 콜백 분리를 위한 전용 NodeHandle + 큐
  ros::NodeHandle nh_video_;  ///< 영상 전용 콜백용
  ros::NodeHandle nh_data_;   ///< TOD/SSOV/RAW/서비스 전용

  ros::CallbackQueue video_queue_; ///< 영상 토픽용 큐
  ros::CallbackQueue data_queue_;  ///< 나머지 데이터용 큐

  std::unique_ptr<ros::AsyncSpinner> video_spinner_; ///< 영상 콜백 전용 스레드(1)
  std::unique_ptr<ros::AsyncSpinner> data_spinner_;  ///< 기타 콜백용 스레드(여러 개 가능)

  // ==== ROS Subscribers / Publishers ====
  ros::Subscriber sub_tx;
  ros::Subscriber sub_tod_status;
  ros::Publisher  pub_tod_status;
  ros::Publisher  pub_tcp_status;      // tcpip_msgs::status
  ros::Publisher  pub_tcp_rx_;         // raw bytes (monitor)
  ros::Publisher  pub_rx_payload_;     // PVD/ControlCmd
  ros::Publisher  pub_v2x_stat_;       // nr_v2x_msgs::V2XStat (latency/dist/PDR)

  std::vector<ros::Subscriber> sub_videos_;
  std::vector<ros::Publisher>  pubs_rx_video_;   // 스트림별 퍼블리셔

  ros::Timer      status_timer_;
  ros::Timer      ssov_timer_;

  // services
  ros::ServiceServer srv_wsr_tx_;
  ros::ServiceServer srv_resolve_ip_;
  ros::ServiceServer srv_start_tcp_server_;
  ros::ServiceServer srv_stop_tcp_server_;
  ros::ServiceServer srv_start_tcp_client_;
  ros::ServiceServer srv_stop_tcp_client_;
  ros::ServiceServer srv_open_udp_socket_;
  ros::ServiceServer srv_close_socket_;
  ros::ServiceServer srv_v2x_config_;

  // status / db base
  ros::Publisher pub_modem_tx_status_;
  ros::Publisher pub_modem_rx_status_;
  ros::Publisher pub_comm_status_;
  ros::Publisher pub_ctrl_status_;
  ros::Publisher pub_db_base_;

  // ==== IO ====
  boost::asio::io_context m_io_context;
  boost::asio::executor_work_guard<boost::asio::io_context::executor_type> m_work_guard;
  std::vector<std::thread> io_threads_;   // run() 스레드 풀

  // 빌드 전용 스레드풀 (필요 시 RAW/SSOV 빌드 분리에 사용)
  std::unique_ptr<boost::asio::thread_pool> build_pool_;
  int  build_threads_{2};

  // RX raw 모니터 토글
  bool debug_rx_raw_{false};

  // 비디오 TLVC 외피 빌더
  std::vector<uint8_t> build_video_wire_(const uint8_t* mux,
                                         size_t mux_len,
                                         uint16_t seq) const;

  // 수신 리어셈블 버퍼(소켓별)
  std::unordered_map<uint32_t, std::vector<uint8_t>> rx_bufs_;

  // ==== dynamic containers ====
  // servers/clients registry
  std::map<uint32_t, std::shared_ptr<tcp_server_t>> m_tcp_servers;
  std::map<uint32_t, std::shared_ptr<tcp_client_t>> m_tcp_clients;

  // open sockets (TCP/UDP 공용)
  std::map<uint32_t, std::shared_ptr<socket_t>>     m_sockets;
  // 송신 경로로 직접 쓰는 TCP 세션 소켓
  std::map<uint32_t, std::shared_ptr<tcp_socket_t>> m_tx_tcp_sockets;

  // link metrics (per-socket id)
  mutable std::map<uint32_t, metrics::link_metrics_t> m_link_metrics;

  // ==== params ====
  std::string overall_s_{"v1"};
  bool   isVehicle_{false};

  // front/left/right 송수신 토픽
  std::string topic_video_[4];   // 0:front, 1:left, 2:right 3:avm
  std::string topic_tx_;

  // TLVC / Extensible
  uint32_t psid_ext_{58200};
  uint8_t  cast_mode_{0};
  uint32_t src_id_{0};
  uint32_t dst_id_{0};
  v2x::crc16::Variant crc_variant_{v2x::crc16::Variant::XMODEM};

  // ===== WSR/WSC 요청-응답 동기화 =====
  struct WscReply {
    uint16_t seq;
    uint8_t  action_result;
    uint32_t psid;
  };
  std::mutex              wsr_mtx_;
  std::condition_variable wsr_cv_;
  std::map<uint16_t, WscReply> wsc_inbox_;

  // 최신 직렬화 스냅샷 (원자적 load/store)
  std::shared_ptr<std::vector<uint8_t>> last_raw_payload_;
  std::shared_ptr<std::vector<uint8_t>> last_tod_status_;

  std::atomic<uint32_t> last_tod_seq_{0};
  std::atomic<uint32_t> last_tod_seq_sent_{0};
  std::atomic_flag      ssov_inflight_ = ATOMIC_FLAG_INIT;

  // DB_V2X meta (SSOV 용)
  v2x::proto::v1::DbV2xMeta db_meta_;

  // video mux (수신측 재조립기)
  v2x::proto::rawmux::RxFragBank mux_bank_;

  // 스트림 개수 (front/left/right)
  static constexpr int kMaxStreams = 4;

  // 영상 조각화/페이싱 파라미터
  uint32_t video_frag_bytes_{1200};   // 조각 payload 바이트 (기본 1200)
  uint32_t video_pacing_us_{200};     // 조각 간 간격(µs)

  // modem latency & distance
  uint64_t last_tx_devts_{0}, last_rx_devts_{0};   // 장비 타임스탬프(10us 단위)
  int32_t  last_tx_lat_{0},  last_tx_lon_{0};
  int32_t  last_rx_lat_{0},  last_rx_lon_{0};
  uint16_t e2e_latency_us{0};
  uint64_t e2e_latency_us_accum_{0};

  // --- SSOV PDR
  mutable std::mutex ssov_pdr_mtx_;
  PdrCounter ssov_pdr_; 

  // --- VIDEO PDR
  std::array<std::mutex, kMaxStreams> video_pdr_mtx_;
  std::array<PdrCounter, kMaxStreams> video_pdr_;
  std::array<ros::Time,  kMaxStreams> video_last_seen_;
  int video_idle_reset_ms_{1000};

  // seq
  std::atomic<uint16_t> ssov_seq16_{0};
  std::atomic<uint16_t> video_hdr_seq16_{0};
  std::atomic<uint16_t> raw_seq16_{0};
  std::atomic<uint16_t> wsr_seq16_{0};
  std::atomic<uint32_t> video_seq_{0};
};

} // namespace v2x_interface
