#include "nr_v2x_interface/endpoint.hpp"
#include <algorithm>
#include <array>
#include <cstring>
#include <sstream>

using boost::asio::ip::address_v4;
using boost::asio::ip::tcp;
using boost::asio::ip::udp;

namespace v2x_interface { namespace endpoint {

static inline address_v4 v4_from_ros(const tcpip_msgs::endpoint& ep) noexcept {
  address_v4::bytes_type bytes{}; // 0-init
  std::copy_n(ep.ip.begin(), bytes.size(), bytes.begin());
  return address_v4(bytes);
}
static inline void ros_set_v4_bytes(tcpip_msgs::endpoint& out, const address_v4& a) {
  const auto bytes = a.to_bytes();
  std::copy_n(bytes.begin(), bytes.size(), out.ip.begin());
}

// ---- TCP ----
tcp::endpoint to_asio_tcp(const tcpip_msgs::endpoint& ep_ros) noexcept {
  return tcp::endpoint(v4_from_ros(ep_ros), ep_ros.port);
}
tcpip_msgs::endpoint to_ros(const tcp::endpoint& ep_asio) noexcept {
  tcpip_msgs::endpoint ep_ros{};
  const auto addr = ep_asio.address();
  if (!addr.is_v4()) {
    std::memset(ep_ros.ip.data(), 0, ep_ros.ip.size());
  } else {
    ros_set_v4_bytes(ep_ros, addr.to_v4());
  }
  ep_ros.port = ep_asio.port();
  return ep_ros;
}

// ---- UDP ----
udp::endpoint to_asio_udp(const tcpip_msgs::endpoint& ep_ros) noexcept {
  return udp::endpoint(v4_from_ros(ep_ros), ep_ros.port);
}
tcpip_msgs::endpoint to_ros(const udp::endpoint& ep_asio) noexcept {
  tcpip_msgs::endpoint ep_ros{};
  const auto addr = ep_asio.address();
  if (!addr.is_v4()) {
    std::memset(ep_ros.ip.data(), 0, ep_ros.ip.size());
  } else {
    ros_set_v4_bytes(ep_ros, addr.to_v4());
  }
  ep_ros.port = ep_asio.port();
  return ep_ros;
}

// ---- String helpers ----
std::string to_string(const tcpip_msgs::endpoint& ep) {
  const auto a = v4_from_ros(ep).to_bytes();
  std::ostringstream oss;
  oss << int(a[0]) << '.' << int(a[1]) << '.' << int(a[2]) << '.' << int(a[3]) << ':' << ep.port;
  return oss.str();
}
bool parse(const std::string& text, tcpip_msgs::endpoint& out) noexcept {
  auto pos = text.rfind(':'); if (pos == std::string::npos) return false;
  std::string ip = text.substr(0, pos), ps = text.substr(pos+1);

  char* endp=nullptr; long port = std::strtol(ps.c_str(), &endp, 10);
  if (!endp || *endp!='\0' || port<0 || port>65535) return false;

  std::array<int,4> oct{}; char dot;
  std::istringstream iss(ip);
  if (!(iss>>oct[0]>>dot) || dot!='.') return false;
  if (!(iss>>oct[1]>>dot) || dot!='.') return false;
  if (!(iss>>oct[2]>>dot) || dot!='.') return false;
  if (!(iss>>oct[3]) || (iss.peek()!=std::char_traits<char>::eof())) return false;
  for (int v:oct) if (v<0 || v>255) return false;

  out.ip = {{ uint8_t(oct[0]), uint8_t(oct[1]), uint8_t(oct[2]), uint8_t(oct[3]) }};
  out.port = uint16_t(port);
  return true;
}
std::string to_string(const tcp::endpoint& ep){ return to_string(to_ros(ep)); }
std::string to_string(const udp::endpoint& ep){ return to_string(to_ros(ep)); }

// ---- Generic by protocol_t (C++14: 오버로드로 제공) ----
tcp::endpoint to_asio(protocol_t /*proto*/, const tcpip_msgs::endpoint& ros_ep,
                      std::enable_if_t<true, tcp::endpoint>*) noexcept {
  return to_asio_tcp(ros_ep);
}
udp::endpoint to_asio(protocol_t /*proto*/, const tcpip_msgs::endpoint& ros_ep,
                      std::enable_if_t<true, udp::endpoint>*) noexcept {
  return to_asio_udp(ros_ep);
}
tcpip_msgs::endpoint to_ros(protocol_t /*proto*/, const tcp::endpoint& ep) noexcept { return to_ros(ep); }
tcpip_msgs::endpoint to_ros(protocol_t /*proto*/, const udp::endpoint& ep) noexcept { return to_ros(ep); }

}} // ns
