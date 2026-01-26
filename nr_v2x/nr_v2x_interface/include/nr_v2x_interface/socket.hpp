#pragma once
/// \file socket.hpp
/// \brief Base socket with protocol + app-channel tagging and light introspection.

#include "nr_v2x_interface/protocol.hpp"
#include <tcpip_msgs/endpoint.h>
#include <cstdint>
#include <string>
#include <atomic>

namespace v2x_interface {

/// \brief 애플리케이션 용도 채널 구분 (PVD 1 + VIDEO 3)
enum class app_channel_t : std::uint8_t {
  UNKNOWN = 0xFF,
  PVD     = 0,   ///< Probe Vehicle Data
  VIDEO0  = 1,   ///< 영상 스트림 #0
  VIDEO1  = 2,   ///< 영상 스트림 #1
  VIDEO2  = 3    ///< 영상 스트림 #2
};

inline const char* to_cstr(app_channel_t c) noexcept {
  switch (c) {
    case app_channel_t::PVD:    return "PVD";
    case app_channel_t::VIDEO0: return "VIDEO0";
    case app_channel_t::VIDEO1: return "VIDEO1";
    case app_channel_t::VIDEO2: return "VIDEO2";
    default:                    return "UNKNOWN";
  }
}

class socket_t {
public:
  explicit socket_t(std::uint32_t id, protocol_t protocol,
                    app_channel_t channel = app_channel_t::UNKNOWN) noexcept;
  virtual ~socket_t() noexcept = default;

  socket_t(const socket_t&)            = delete;
  socket_t& operator=(const socket_t&) = delete;
  socket_t(socket_t&&) noexcept        = default;
  socket_t& operator=(socket_t&&) noexcept = default;

  /// \brief Close the socket (idempotent in derived classes).
  virtual void close() = 0;

  [[nodiscard]] protocol_t    protocol() const noexcept { return m_protocol; }
  [[nodiscard]] std::uint32_t id() const noexcept       { return m_id; }
  [[nodiscard]] app_channel_t channel() const noexcept  { return m_channel.load(std::memory_order_relaxed); }
  /// \brief 채널 변경(수명 초기에만 호출 권장)
  void set_channel(app_channel_t ch) noexcept { m_channel.store(ch, std::memory_order_relaxed); }

  /// \brief 현재 소켓이 열려있는가?
  [[nodiscard]] virtual bool is_open() const = 0;

  /// \brief 로깅/상태용 라벨: "TCP#12/VIDEO1" 형태
  [[nodiscard]] virtual std::string label() const;
    
  /// \brief 로컬/리모트 엔드포인트 조회(가능한 경우 true 반환)
  [[nodiscard]] virtual bool local_endpoint(tcpip_msgs::endpoint& out)  const noexcept { (void)out; return false; }
  [[nodiscard]] virtual bool remote_endpoint(tcpip_msgs::endpoint& out) const noexcept { (void)out; return false; }

protected:
  const std::uint32_t m_id;
  const protocol_t    m_protocol;

  // 채널은 runtime에 바뀔 수 있으므로 atomic
  std::atomic<app_channel_t> m_channel;
};

} // namespace v2x_interface
