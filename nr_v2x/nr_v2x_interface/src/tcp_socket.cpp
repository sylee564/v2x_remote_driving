#include "nr_v2x_interface/tcp_socket.hpp"
#include <ros/console.h>
#include <algorithm>
#include <cstring>

namespace v2x_interface {

tcp_socket_t::tcp_socket_t(std::uint32_t id,
                           app_channel_t channel,
                           boost::asio::io_context& io,
                           socket_ptr s) noexcept
: socket_t(id, protocol_t::TCP, channel)
, m_io(io)
, m_strand(v2x_asio::make_strand_compat(io))
, m_sock(std::move(s))
{
  if (m_sock && m_sock->is_open()) {
    m_alive.store(true, std::memory_order_release);
  } else {
    m_alive.store(false, std::memory_order_release);
  }
}

tcp_socket_t::~tcp_socket_t() {
  try {
    do_close({});
  } catch (...) {
    // 파괴 시에는 예외 무시
  }
}

void tcp_socket_t::start(bytes_cb_t on_bytes, closed_cb_t on_closed) noexcept {
  auto self = shared_from_this();
  v2x_asio::post_compat(
    m_strand,
    [self,
     on_bytes = std::move(on_bytes),
     on_closed = std::move(on_closed)]() mutable noexcept
    {
      self->do_start(std::move(on_bytes), std::move(on_closed));
    }
  );
}

void tcp_socket_t::do_start(bytes_cb_t on_bytes, closed_cb_t on_closed) noexcept {
  if (m_closed || !m_sock || !m_sock->is_open()) {
    m_alive.store(false, std::memory_order_release);
    return;
  }

  m_on_bytes  = std::move(on_bytes);
  m_on_closed = std::move(on_closed);

  boost::system::error_code ec;
  m_sock->set_option(boost::asio::ip::tcp::no_delay(true), ec);             ec.clear();
  m_sock->set_option(boost::asio::socket_base::keep_alive(true), ec);      ec.clear();
  m_sock->set_option(boost::asio::socket_base::send_buffer_size(256*1024), ec);   ec.clear();
  m_sock->set_option(boost::asio::socket_base::receive_buffer_size(256*1024), ec); ec.clear();

  m_alive.store(true, std::memory_order_release);

  ROS_INFO_STREAM(label() << " started"
    << " local="  << endpoint::to_string(endpoint::to_ros(m_sock->local_endpoint(ec)))
    << " remote=" << endpoint::to_string(endpoint::to_ros(m_sock->remote_endpoint(ec))));

  kick_read();
}

void tcp_socket_t::kick_read() noexcept {
  if (m_closed || !m_sock) return;

  auto self = shared_from_this();
  m_sock->async_read_some(
    boost::asio::buffer(m_rbuf.data(), m_rbuf.size()),
    v2x_asio::bind_exec_compat(
      m_strand,
      [self](const boost::system::error_code& ec, std::size_t n) noexcept {
        self->on_read(ec, n);
      }
    )
  );
}

void tcp_socket_t::on_read(const boost::system::error_code& ec, std::size_t n) noexcept {
  if (m_closed) return;

  if (ec) {
    if (ec != boost::asio::error::operation_aborted) {
      // 필요하면 다시 활성화
      // ROS_WARN_STREAM(label() << " read error: " << ec.message());
    }
    do_close(ec);
    return;
  }

  if (n > 0 && m_on_bytes) {
    try {
      m_on_bytes(m_rbuf.data(), n);
    } catch (const std::exception& /*e*/) {
      // 필요하면 로그
      // ROS_ERROR_STREAM(label() << " on_bytes threw: " << e.what());
    }
  }

  kick_read();
}

// 편의 함수: 내부적으로 shared_ptr로 감싸서 enqueue_write 사용
void tcp_socket_t::write(const uint8_t* data, std::size_t n) noexcept {
  if (!data || n == 0) return;
  if (!m_alive.load(std::memory_order_acquire)) return;

  auto buf = std::make_shared<std::vector<uint8_t>>(data, data + n);
  enqueue_write(buf);
}

// strand 안에서만 TX 큐 조작 + kick_write 호출
void tcp_socket_t::enqueue_write(std::shared_ptr<std::vector<uint8_t>> buf) noexcept {
  if (!buf || buf->empty()) return;
  if (!m_alive.load(std::memory_order_acquire)) return;

  auto self = shared_from_this();

  v2x_asio::post_compat(
    m_strand,
    [self, buf]() noexcept {
      if (self->m_closed || !self->m_sock) return;

      const std::size_t sz = buf->size();

      // HARD LIMIT: 큐 전체 + 새 패킷 크기가 hard limit을 넘으면 드롭 정책 적용
      if (self->m_q_bytes + sz > self->m_q_bytes_hard_limit) {
        if (!self->m_wq.empty()) {
          self->m_hard_drops++;
          ROS_WARN_THROTTLE(
            1.0,
            "%s enqueue_write HARD limit exceeded (%.2f MB > %.2f MB). "
            "Dropping oldest buffered packet (hard_drops=%zu).",
            self->label().c_str(),
            (self->m_q_bytes + sz) / (1024.0 * 1024.0),
            self->m_q_bytes_hard_limit / (1024.0 * 1024.0),
            self->m_hard_drops
          );
          self->m_q_bytes -= self->m_wq.front()->size();
          self->m_wq.pop_front();
        } else {
          self->m_hard_drops++;
          ROS_WARN_THROTTLE(
            1.0,
            "%s enqueue_write HARD limit exceeded with empty queue. "
            "Dropping new packet (size=%.2f KB, hard_drops=%zu).",
            self->label().c_str(),
            sz / 1024.0,
            self->m_hard_drops
          );
          return;
        }
      }

      self->m_q_bytes += sz;
      self->m_wq.emplace_back(buf);

      if (self->m_q_bytes > self->m_q_bytes_soft_limit) {
        self->m_soft_drops++;
        ROS_WARN_THROTTLE(
          5.0,
          "%s enqueue_write SOFT limit exceeded: %.2f MB (soft_limit=%.2f MB, soft_events=%zu).",
          self->label().c_str(),
          self->m_q_bytes / (1024.0 * 1024.0),
          self->m_q_bytes_soft_limit / (1024.0 * 1024.0),
          self->m_soft_drops
        );
      }

      if (!self->m_writing) {
        self->kick_write();   // 반드시 strand 안에서만 호출
      }
    }
  );
}

// kick_write도 strand 안에서만 호출되어야 함
void tcp_socket_t::kick_write() noexcept {
  if (m_closed || !m_sock) return;
  if (m_writing || m_wq.empty()) return;

  m_writing = true;

  // 배치 쓰기 (syscall 감소)
  constexpr std::size_t LIMIT = 128 * 1024;  // 128KB
  auto batch = std::make_shared<std::vector<uint8_t>>();
  batch->reserve(LIMIT);

  std::size_t total = 0;
  while (!m_wq.empty() && (total + m_wq.front()->size()) <= LIMIT) {
    auto& front = m_wq.front();
    batch->insert(batch->end(), front->begin(), front->end());
    total += front->size();
    m_q_bytes -= front->size();
    m_wq.pop_front();
  }

  // 만약 front 하나가 LIMIT보다 큰 경우 대비
  if (batch->empty() && !m_wq.empty()) {
    auto first = std::move(m_wq.front());
    m_wq.pop_front();
    m_q_bytes -= first->size();
    *batch = std::move(*first);
  }

  if (batch->empty()) {
    m_writing = false;
    return;
  }

  auto self = shared_from_this();
  boost::asio::async_write(
    *m_sock,
    boost::asio::buffer(*batch),
    v2x_asio::bind_exec_compat(
      m_strand,
      [self, batch](const boost::system::error_code& ec, std::size_t /*sent*/) noexcept {
        if (ec && ec != boost::asio::error::operation_aborted) {
          // 필요하면 로그 활성화
          // ROS_WARN_STREAM(self->label() << " write error: " << ec.message());
          self->do_close(ec);
          return;
        }
        self->m_writing = false;
        if (!self->m_wq.empty()) {
          self->kick_write();  // 다시 큐에 남아 있으면 이어서 전송
        }
      }
    )
  );
}

void tcp_socket_t::close() {
  if (m_closed) return;
  try {
    auto self = shared_from_this();
    v2x_asio::post_compat(
      m_strand,
      [self]() noexcept {
        self->do_close({});
      }
    );
  } catch (const std::bad_weak_ptr&) {
    // 이미 shared_ptr 수명 끝나가는 경우 방어
    do_close({});
  }
}

void tcp_socket_t::do_close(const boost::system::error_code& reason) noexcept {
  if (m_closed) return;
  m_closed = true;
  m_alive.store(false, std::memory_order_release);

  if (m_sock) {
    boost::system::error_code ec;
    m_sock->shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
    m_sock->close(ec);
    m_sock.reset();
  }

  m_wq.clear();
  m_q_bytes = 0;
  m_writing = false;

  if (m_on_closed) {
    try {
      m_on_closed(reason);
    } catch (const std::exception& /*e*/) {
      // 필요 시 로그 가능
      // ROS_ERROR_STREAM(label() << " on_closed threw: " << e.what());
    }
  }
}

bool tcp_socket_t::is_open() const {
  return m_alive.load(std::memory_order_acquire)
      && m_sock
      && m_sock->is_open();
}

bool tcp_socket_t::local_endpoint(tcpip_msgs::endpoint& out) const noexcept {
  if (!m_sock) return false;
  boost::system::error_code ec;
  auto lep = m_sock->local_endpoint(ec);
  if (ec) return false;
  out = endpoint::to_ros(lep);
  return true;
}

bool tcp_socket_t::remote_endpoint(tcpip_msgs::endpoint& out) const noexcept {
  if (!m_sock) return false;
  boost::system::error_code ec;
  auto rep = m_sock->remote_endpoint(ec);
  if (ec) return false;
  out = endpoint::to_ros(rep);
  return true;
}

tcpip_msgs::tcp_socket tcp_socket_t::description() const noexcept {
  tcpip_msgs::tcp_socket d;
  d.id = m_id;
  tcpip_msgs::endpoint lep{}, rep{};
  if (local_endpoint(lep))  d.local_endpoint  = lep;
  if (remote_endpoint(rep)) d.remote_endpoint = rep;
  return d;
}

} // namespace v2x_interface
