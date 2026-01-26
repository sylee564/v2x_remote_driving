#include "nr_v2x_interface/tcp_client.hpp"
#include "nr_v2x_interface/endpoint.hpp"
#include <ros/console.h>
#include <chrono>
#include <cstring>

namespace {
inline bool is_any_endpoint(const tcpip_msgs::endpoint& ep) noexcept {
  return ep.port == 0 && ep.ip[0]==0 && ep.ip[1]==0 && ep.ip[2]==0 && ep.ip[3]==0;
}
inline bool port_valid(std::uint16_t p) noexcept { return p != 0; }
} // anon

namespace v2x_interface {

tcp_client_t::tcp_client_t(boost::asio::io_context& io, std::uint32_t id, connection_cb_t cb) noexcept
  : m_io(io)
  , m_strand(v2x_asio::make_strand_compat(io))
  , m_id(id)
  , m_timer(io)
  , m_callback_connection(std::move(cb))
{}

tcp_client_t::~tcp_client_t() { stop(); }

bool tcp_client_t::start(const tcpip_msgs::endpoint& local,
                         const tcpip_msgs::endpoint& remote) noexcept {
  if (m_connect_inflight.load(std::memory_order_acquire) || m_active.load(std::memory_order_acquire)) {
    ROS_ERROR_STREAM("tcp client " << m_id << " failed to start (already active or connecting)");
    return false;
  }
  if (!port_valid(remote.port)) {
    ROS_ERROR_STREAM("tcp client " << m_id << " failed to start (remote port is 0)");
    return false;
  }
  try {
    auto self = shared_from_this();
    boost::asio::post(m_strand, [self, local, remote]() noexcept {
      // ✅ 재시작 허용
      self->m_stopped = false;
      self->do_start(local, remote);
    });
    return true;
  } catch (...) {
    ROS_ERROR_STREAM("tcp client " << m_id << " must be owned by std::shared_ptr before start()");
    return false;
  }
}

void tcp_client_t::stop() noexcept {
  try {
    auto self = shared_from_this();
    boost::asio::post(m_strand, [self]() noexcept { self->do_stop(); });
  } catch (...) {
    // 파괴 경로일 수 있으니 무시
  }
}

void tcp_client_t::do_start(const tcpip_msgs::endpoint& local,
                            const tcpip_msgs::endpoint& remote) noexcept {
  if (m_stopped) return;

  // 소켓 생성
  try {
    m_socket = std::make_shared<socket_t>(m_io);
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("tcp client " << m_id << " failed to allocate socket (" << e.what() << ")");
    return;
  }

  boost::system::error_code ec;
  m_socket->open(boost::asio::ip::tcp::v4(), ec);
  if (ec) {
    ROS_ERROR_STREAM("tcp client " << m_id << " failed to open socket (" << ec.message() << ")");
    m_socket.reset();
    return;
  }

  // 빠른 재시작 허용
  m_socket->set_option(boost::asio::socket_base::reuse_address(true), ec); ec.clear();

  // 바인드(요청 시)
  if (!is_any_endpoint(local)) {
    m_socket->bind(v2x_interface::endpoint::to_asio_tcp(local), ec);
    if (ec) {
      ROS_ERROR_STREAM("tcp client " << m_id << " failed to bind (" << ec.message() << ")");
      m_socket.reset();
      return;
    }
  }

  m_local_endpoint  = local;
  m_remote_endpoint = remote;

  m_connect_inflight.store(true, std::memory_order_release);
  m_active.store(true, std::memory_order_release);

  const auto remote_ep = v2x_interface::endpoint::to_asio_tcp(remote);

  auto self = shared_from_this();
  m_socket->async_connect(
      remote_ep,
      v2x_asio::bind_exec_compat(m_strand,
        [self](const boost::system::error_code& ec_cb) noexcept { self->connect_callback(ec_cb); }
      )
  );

  // 타임아웃 타이머 시작
  m_timer.expires_after(std::chrono::milliseconds(m_connect_timeout_ms));
  m_timer.async_wait(
      v2x_asio::bind_exec_compat(m_strand,
        [self](const boost::system::error_code& ec_t) noexcept { self->on_connect_timeout(ec_t); }
      )
  );

  ROS_INFO_STREAM("tcp client " << m_id << " connecting to " << endpoint::to_string(remote));
}

void tcp_client_t::do_stop() noexcept {
  if (m_stopped) return;
  m_stopped = true;

  boost::system::error_code ec;
  m_timer.cancel(ec);

  if (m_socket) {
    if (m_socket->is_open()) {
      boost::system::error_code ec2;
      m_socket->shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec2);
      m_socket->close(ec2);
    }
    m_socket.reset();
  }

  m_connect_inflight.store(false, std::memory_order_release);
  m_active.store(false, std::memory_order_release);
  ROS_INFO_STREAM("tcp client " << m_id << " stopped");
}

tcpip_msgs::tcp_client tcp_client_t::description() const noexcept {
  tcpip_msgs::tcp_client d;
  d.id = m_id;
  d.local_endpoint  = m_local_endpoint;
  d.remote_endpoint = m_remote_endpoint;
  return d;
}

void tcp_client_t::apply_socket_options(const socket_ptr& s) noexcept {
  if (!s) return;
  boost::system::error_code ec;
  s->set_option(boost::asio::ip::tcp::no_delay(true), ec); ec.clear();
  s->set_option(boost::asio::socket_base::keep_alive(true), ec); ec.clear();

  // 선택:
  // s->set_option(boost::asio::socket_base::linger(true, 0), ec); ec.clear();
  // s->set_option(boost::asio::socket_base::send_buffer_size(1<<20), ec); ec.clear();
  // s->set_option(boost::asio::socket_base::receive_buffer_size(1<<20), ec); ec.clear();
}

void tcp_client_t::connect_callback(const boost::system::error_code& ec) noexcept {
  // 타이머 취소
  boost::system::error_code ec_cancel;
  m_timer.cancel(ec_cancel);

  if (m_stopped) return;

  if (ec) {
    if (ec == boost::asio::error::operation_aborted) {
      ROS_WARN_STREAM("tcp client " << m_id << " connect aborted");
    } else {
      ROS_ERROR_STREAM("tcp client " << m_id << " failed to connect (" << ec.message() << ")");
    }
    do_stop();
    return;
  }

  apply_socket_options(m_socket);
  ROS_INFO_STREAM("tcp client " << m_id << " connected: local="
                  << endpoint::to_string(m_local_endpoint)
                  << " remote=" << endpoint::to_string(m_remote_endpoint));

  // 연결 소켓을 상위로 넘기고, 자신은 정리(드라이버가 inactive 클린업)
  auto handed = m_socket;
  m_socket.reset();
  m_connect_inflight.store(false, std::memory_order_release);
  m_active.store(false, std::memory_order_release);

  try {
    if (m_callback_connection) m_callback_connection(std::move(handed));
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("tcp client " << m_id << " connection callback threw: " << e.what());
  }
}

void tcp_client_t::on_connect_timeout(const boost::system::error_code& ec) noexcept {
  if (m_stopped) return;
  if (ec) return; // 취소됨
  if (!m_connect_inflight.load(std::memory_order_acquire)) return;

  ROS_ERROR_STREAM("tcp client " << m_id << " connect timeout after " << m_connect_timeout_ms << "ms");
  if (m_socket) { boost::system::error_code ec2; m_socket->close(ec2); }
  // close()되면 async_connect는 에러 콜백으로 리턴되어 위에서 정리됨
}

} // namespace v2x_interface
