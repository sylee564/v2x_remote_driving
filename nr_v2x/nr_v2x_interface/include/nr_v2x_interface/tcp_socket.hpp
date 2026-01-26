#pragma once
/// \file tcp_socket.hpp
/// \brief v2x_interface::tcp_socket_t - strand 직렬화된 TCP 소켓 래퍼 (read loop + write queue + batching)

#include <ros/ros.h>

#include "nr_v2x_interface/socket.hpp"
#include "nr_v2x_interface/detail/asio_compat.hpp"
#include "nr_v2x_interface/endpoint.hpp"

#include <tcpip_msgs/tcp_socket.h>

#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/write.hpp>
#include <atomic>
#include <functional>
#include <memory>
#include <vector>
#include <array>
#include <deque>
#include <chrono>

namespace v2x_interface {

class tcp_socket_t final : public socket_t,
                           public std::enable_shared_from_this<tcp_socket_t> {
public:
  using asio_socket   = boost::asio::ip::tcp::socket;
  using socket_ptr    = std::shared_ptr<asio_socket>;
  using bytes_cb_t    = std::function<void(const uint8_t* data, std::size_t n)>;
  using closed_cb_t   = std::function<void(const boost::system::error_code& ec)>;

  tcp_socket_t(std::uint32_t id,
               app_channel_t channel,
               boost::asio::io_context& io,
               socket_ptr s) noexcept;

  ~tcp_socket_t() override;

  // lifecycle
  void start(bytes_cb_t on_bytes, closed_cb_t on_closed) noexcept;
  void close();

  // io
  //  - write: 편의 함수 (즉시 shared_ptr로 감싸서 enqueue_write 호출)
  //  - enqueue_write: strand 안에서 큐 적재 → 배치 async_write
  void write(const uint8_t* data, std::size_t n) noexcept;
  void enqueue_write(std::shared_ptr<std::vector<uint8_t>> buf) noexcept;

  // info
  bool is_open() const override;
  bool local_endpoint(tcpip_msgs::endpoint& out) const noexcept override;
  bool remote_endpoint(tcpip_msgs::endpoint& out) const noexcept override;
  tcpip_msgs::tcp_socket description() const noexcept;

private:
  void do_start(bytes_cb_t on_bytes, closed_cb_t on_closed) noexcept;
  void kick_read() noexcept;
  void on_read(const boost::system::error_code& ec, std::size_t n) noexcept;

  // 아래 두 함수는 반드시 strand(m_strand) 안에서만 호출되어야 함
  void kick_write() noexcept;
  void do_close(const boost::system::error_code& reason) noexcept;

private:
  boost::asio::io_context& m_io;
  v2x_asio::strand_t       m_strand;
  socket_ptr               m_sock;

  // callbacks (strand 보호)
  bytes_cb_t               m_on_bytes;
  closed_cb_t              m_on_closed;

  // rx
  std::array<uint8_t, 64 * 1024> m_rbuf{};

  // tx (strand 보호)
  std::deque<std::shared_ptr<std::vector<uint8_t>>> m_wq;
  bool        m_writing{false};
  std::size_t m_q_bytes{0};
  std::size_t m_q_bytes_soft_limit{4 * 1024 * 1024};   // 4MB: 소프트 제한(경고)
  std::size_t m_q_bytes_hard_limit{16 * 1024 * 1024};  // 16MB: 하드 제한(드롭)

  // drop stats (디버그 용)
  std::size_t m_soft_drops{0};   // soft limit 경고 횟수 (큐가 너무 자주 커지는지 보기 용)
  std::size_t m_hard_drops{0};   // hard limit에서 실제로 패킷 드롭한 횟수

  // state
  std::atomic<bool>  m_alive{true};    // 외부 빠른 체크용 (is_open 등)
  bool               m_closed{false};  // strand 보호
};

} // namespace v2x_interface
