#pragma once
/// \file tcp_server.hpp
/// \brief Defines the v2x_interface::tcp_server_t class.
#ifndef V2X_INTERFACE_TCP_SERVER_HPP
#define V2X_INTERFACE_TCP_SERVER_HPP

#include <tcpip_msgs/endpoint.h>
#include <tcpip_msgs/tcp_server.h>

#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/steady_timer.hpp>
#include "nr_v2x_interface/detail/asio_compat.hpp"

#include <functional>
#include <memory>
#include <atomic>
#include <cstdint>

namespace v2x_interface {

/// \brief A TCP server for accepting incoming connections.
/// \note Create with std::make_shared and then call start().
class tcp_server_t : public std::enable_shared_from_this<tcp_server_t>
{
public:
    using socket_t        = boost::asio::ip::tcp::socket;
    using socket_ptr      = std::shared_ptr<socket_t>;
    using connection_cb_t = std::function<void(socket_ptr)>;

    // CONSTRUCTORS
    tcp_server_t(boost::asio::io_context& io_context,
                 std::uint32_t id,
                 connection_cb_t connection_callback) noexcept;
    ~tcp_server_t();

    // CONTROL
    /// \brief Starts the TCP server.
    /// \param local_endpoint The local endpoint to start the server on.
    /// \returns TRUE if the start request was posted, otherwise FALSE.
    bool start(const tcpip_msgs::endpoint& local_endpoint) noexcept;

    /// \brief Stops the TCP server. Thread-safe.
    void stop() noexcept;

    // CONFIG (호출 타이밍: start 이전 권장)
    void set_backlog(int backlog) noexcept { m_listen_backlog = backlog; }
    void set_reuse_port(bool en) noexcept { m_reuse_port = en; }
    void set_accept_retry_ms(std::uint32_t ms) noexcept { m_accept_retry_ms = ms; }

    // PROPERTIES
    [[nodiscard]] tcpip_msgs::tcp_server description() const noexcept;
    [[nodiscard]] bool is_running() const noexcept { return m_running.load(std::memory_order_acquire); }
    [[nodiscard]] tcpip_msgs::endpoint local_endpoint() const noexcept { return m_local_endpoint; }

private:
    // STRAND-ONLY HELPERS
    void do_start(const tcpip_msgs::endpoint& local_endpoint) noexcept;
    void do_stop() noexcept;

    void async_accept() noexcept; // posts next accept (strand)
    void on_accept(const socket_ptr& sock, const boost::system::error_code& ec) noexcept;
    void apply_socket_options(const socket_ptr& s) noexcept;

    void schedule_accept_retry() noexcept;

private:
    // EXECUTION CONTEXT
    boost::asio::io_context&  m_io;
    v2x_asio::executor_t      m_exec;
    v2x_asio::strand_t        m_strand;

    // STATE
    const std::uint32_t  m_id;
    std::atomic<bool>    m_running{false}; // quick check (updates on strand)
    bool                 m_stopped{false}; // strand-protected

    // ACCEPTOR
    boost::asio::ip::tcp::acceptor m_acceptor;

    // ENDPOINT / CALLBACK
    tcpip_msgs::endpoint m_local_endpoint{};
    connection_cb_t      m_callback;

    // CONFIG
    int                  m_listen_backlog{128};
    bool                 m_reuse_port{false};
    std::uint32_t        m_accept_retry_ms{200}; // 에러 시 accept 재시도 간격

    // RETRY TIMER
    boost::asio::steady_timer m_retry_timer;
};

} // namespace v2x_interface

#endif // V2X_INTERFACE_TCP_SERVER_HPP
