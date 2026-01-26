#pragma once
#include <tcpip_msgs/endpoint.h>
#include <tcpip_msgs/tcp_client.h>

#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/steady_timer.hpp>
#include "nr_v2x_interface/detail/asio_compat.hpp"

#include <functional>
#include <memory>
#include <atomic>
#include <cstdint>

namespace v2x_interface {

class tcp_client_t : public std::enable_shared_from_this<tcp_client_t> {
public:
  using socket_t   = boost::asio::ip::tcp::socket;
  using socket_ptr = std::shared_ptr<socket_t>;
  using connection_cb_t = std::function<void(socket_ptr)>;

  tcp_client_t(boost::asio::io_context& io, std::uint32_t id, connection_cb_t cb) noexcept;
  ~tcp_client_t();

  // 비동기 시작: 성공시 true(즉시), 실제 결과는 콜백으로 통지
  bool start(const tcpip_msgs::endpoint& local,
             const tcpip_msgs::endpoint& remote) noexcept;

  // 비동기 정지(재호출 안전)
  void stop() noexcept;

  [[nodiscard]] tcpip_msgs::tcp_client description() const noexcept;
  [[nodiscard]] bool is_active() const noexcept { return m_active.load(std::memory_order_acquire); }

  void set_connect_timeout_ms(std::uint32_t ms) noexcept { m_connect_timeout_ms = ms; }

private:
  void do_start(const tcpip_msgs::endpoint& local,
                const tcpip_msgs::endpoint& remote) noexcept;
  void do_stop() noexcept;

  void connect_callback(const boost::system::error_code& ec) noexcept;
  void on_connect_timeout(const boost::system::error_code& ec) noexcept;
  void apply_socket_options(const socket_ptr& s) noexcept;

private:
  boost::asio::io_context&  m_io;
  v2x_asio::strand_t        m_strand;

  const std::uint32_t m_id;
  std::atomic<bool>   m_active{false};           // connecting 동안 true, 완료/정지 시 false
  std::atomic<bool>   m_connect_inflight{false}; // async_connect 진행 중
  bool                m_stopped{false};          // strand 전용

  tcpip_msgs::endpoint m_local_endpoint{};
  tcpip_msgs::endpoint m_remote_endpoint{};

  socket_ptr                m_socket;
  boost::asio::steady_timer m_timer;
  std::uint32_t             m_connect_timeout_ms{5000};

  connection_cb_t m_callback_connection;
};

} // namespace v2x_interface
