/// \file endpoint.hpp
/// \brief Defines endpoint conversion functions and helpers.
#ifndef V2X_INTERFACE_ENDPOINT_HPP
#define V2X_INTERFACE_ENDPOINT_HPP

#include <string>
#include <tcpip_msgs/endpoint.h>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/ip/udp.hpp>
#include <cstdint>
#include "nr_v2x_interface/protocol.hpp"

namespace v2x_interface {
namespace endpoint {

// ----- TCP/UDP <-> ROS endpoint (IPv4 only) -----
[[nodiscard]] boost::asio::ip::tcp::endpoint to_asio_tcp(const tcpip_msgs::endpoint& endpoint_ros) noexcept;
[[nodiscard]] tcpip_msgs::endpoint           to_ros      (const boost::asio::ip::tcp::endpoint& endpoint_asio) noexcept;

[[nodiscard]] boost::asio::ip::udp::endpoint to_asio_udp(const tcpip_msgs::endpoint& endpoint_ros) noexcept;
[[nodiscard]] tcpip_msgs::endpoint           to_ros      (const boost::asio::ip::udp::endpoint& endpoint_asio) noexcept;

// ----- String helpers -----
// "a.b.c.d:port" 로 변환 (IPv4, 포트 0 허용)
[[nodiscard]] std::string to_string(const tcpip_msgs::endpoint& ep);
// "a.b.c.d:port" 파싱 → ROS endpoint (성공 시 true)
[[nodiscard]] bool parse(const std::string& text, tcpip_msgs::endpoint& out) noexcept;

// TCP/UDP ASIO endpoint → "a.b.c.d:port"
[[nodiscard]] std::string to_string(const boost::asio::ip::tcp::endpoint& ep);
[[nodiscard]] std::string to_string(const boost::asio::ip::udp::endpoint& ep);

// ----- Generic by protocol_t -----
// 프로토콜에 따라 적절한 ASIO endpoint로 변환
[[nodiscard]] boost::asio::ip::tcp::endpoint to_asio(protocol_t proto, const tcpip_msgs::endpoint& ros_ep,
                                                     std::enable_if_t<true, boost::asio::ip::tcp::endpoint>* = nullptr) noexcept;
[[nodiscard]] boost::asio::ip::udp::endpoint to_asio(protocol_t proto, const tcpip_msgs::endpoint& ros_ep,
                                                     std::enable_if_t<true, boost::asio::ip::udp::endpoint>* = nullptr) noexcept;

// ASIO endpoint → ROS endpoint (오버로드로 분기)
[[nodiscard]] tcpip_msgs::endpoint to_ros(protocol_t proto, const boost::asio::ip::tcp::endpoint& ep) noexcept;
[[nodiscard]] tcpip_msgs::endpoint to_ros(protocol_t proto, const boost::asio::ip::udp::endpoint& ep) noexcept;

}} // namespace v2x_interface::endpoint

#endif // V2X_INTERFACE_ENDPOINT_HPP
