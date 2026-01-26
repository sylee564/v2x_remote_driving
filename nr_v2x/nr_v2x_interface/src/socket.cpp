#include "nr_v2x_interface/socket.hpp"
#include <sstream>

namespace v2x_interface {

socket_t::socket_t(std::uint32_t id, protocol_t protocol, app_channel_t channel) noexcept
  : m_id(id), m_protocol(protocol), m_channel(channel) {}

std::string socket_t::label() const {
  std::ostringstream oss;
  oss << (protocol() == protocol_t::TCP ? "TCP" : "UDP")
      << "#" << m_id << "/" << to_cstr(channel());
  return oss.str();
}

} // namespace v2x_interface
