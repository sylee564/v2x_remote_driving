#if defined(SO_REUSEPORT)
#  include <sys/socket.h>
#  include <netinet/in.h>
#endif
#include <chrono>
#include "nr_v2x_interface/tcp_server.hpp"
#include "nr_v2x_interface/endpoint.hpp"

#include <ros/console.h>
#include <boost/asio/socket_base.hpp>
#include <boost/system/error_code.hpp>

namespace v2x_interface {

using tcp = boost::asio::ip::tcp;

// CONSTRUCTORS
tcp_server_t::tcp_server_t(boost::asio::io_context& io_context,
                           std::uint32_t id,
                           connection_cb_t connection_callback) noexcept
    : m_io(io_context)
    , m_exec(io_context.get_executor())
    , m_strand(v2x_asio::make_strand_compat(m_io))
    , m_id(id)
    , m_acceptor(io_context)
    , m_callback(std::move(connection_callback))
    , m_retry_timer(io_context)
{
}

tcp_server_t::~tcp_server_t()
{
    stop(); // 안전한 종료 요청 (strand에서 정리)
}

// CONTROL
bool tcp_server_t::start(const tcpip_msgs::endpoint& local_endpoint) noexcept {
    if (m_running.load(std::memory_order_acquire) || m_acceptor.is_open()) {
        ROS_ERROR_STREAM("tcp server " << m_id << " failed to start (already running)");
        return false;
    }
    try {
        auto self = shared_from_this();
        boost::asio::post(m_strand, [self, local_endpoint]() noexcept {
        self->m_stopped = false; // ✅ 재시작 허용
        self->do_start(local_endpoint);
        });
        return true;
    } catch (...) {
        ROS_ERROR_STREAM("tcp server " << m_id << " must be owned by std::shared_ptr before start()");
        return false;
    }
}

void tcp_server_t::stop() noexcept
{
    try {
        auto self = shared_from_this();
        boost::asio::post(m_strand, [self]() noexcept {
            self->do_stop();
        });
    } catch (...) {
        // 파괴 중일 수 있음. best-effort 무시.
    }
}

// PROPERTIES
tcpip_msgs::tcp_server tcp_server_t::description() const noexcept
{
    tcpip_msgs::tcp_server d;
    d.id = m_id;
    boost::system::error_code ec;
    if (m_acceptor.is_open()) {
        auto lep = m_acceptor.local_endpoint(ec);
        if (!ec) d.local_endpoint = v2x_interface::endpoint::to_ros(lep);
    } else {
        d.local_endpoint = m_local_endpoint;
    }
    return d;
}

// STRAND CONTEXT
void tcp_server_t::do_start(const tcpip_msgs::endpoint& local_endpoint) noexcept
{
    if (m_stopped) return;

    boost::system::error_code ec;

    // open/bind/listen
    m_acceptor.open(tcp::v4(), ec);
    if (ec) {
        ROS_ERROR_STREAM("tcp server " << m_id << " failed to open (" << ec.message() << ")");
        return;
    }

    // 빠른 재시작 허용
    m_acceptor.set_option(boost::asio::socket_base::reuse_address(true), ec); ec.clear();

#ifdef SO_REUSEPORT
    if (m_reuse_port) {
        // 플랫폼에 따라 지원 안될 수 있음: best-effort
        int optval = 1;
        ::setsockopt(m_acceptor.native_handle(), SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));
    }
#endif

    m_acceptor.bind(v2x_interface::endpoint::to_asio_tcp(local_endpoint), ec);
    if (ec) {
        ROS_ERROR_STREAM("tcp server " << m_id << " failed to bind (" << ec.message() << ")");
        boost::system::error_code ec2; m_acceptor.close(ec2);
        return;
    }

    m_acceptor.listen(m_listen_backlog, ec);
    if (ec) {
        ROS_ERROR_STREAM("tcp server " << m_id << " failed to listen (" << ec.message() << ")");
        boost::system::error_code ec2; m_acceptor.close(ec2);
        return;
    }

    m_local_endpoint = local_endpoint;
    m_running.store(true, std::memory_order_release);
    m_stopped = false;

    auto lep = m_acceptor.local_endpoint(ec);
    ROS_INFO_STREAM("tcp server " << m_id << " started on "
                    << (ec ? std::string("0.0.0.0") : lep.address().to_string())
                    << ":" << (ec ? 0 : lep.port()));

    async_accept();
}

void tcp_server_t::do_stop() noexcept
{
    if (m_stopped) return;
    m_stopped = true;
    m_running.store(false, std::memory_order_release);

    boost::system::error_code ec;
    m_retry_timer.cancel(ec);

    if (m_acceptor.is_open()) {
        m_acceptor.cancel(ec); // pending accept 취소
        m_acceptor.close(ec);
    }
    if (ec) {
        ROS_ERROR_STREAM("tcp server " << m_id << " failed to halt (" << ec.message() << ")");
    } else {
        ROS_INFO_STREAM("tcp server " << m_id << " stopped");
    }
}

// ACCEPT LOOP (STRAND)
void tcp_server_t::async_accept() noexcept
{
    if (m_stopped || !m_running.load(std::memory_order_acquire)) return;

    auto next_socket = std::make_shared<tcp::socket>(m_io);

    auto self = shared_from_this();
    m_acceptor.async_accept(
        *next_socket,
        v2x_asio::bind_exec_compat(
            m_strand,
            [self, next_socket](const boost::system::error_code& ec) noexcept {
                self->on_accept(next_socket, ec);
            }
        )
    );
}

void tcp_server_t::on_accept(const socket_ptr& sock, const boost::system::error_code& ec) noexcept
{
    if (m_stopped || !m_running.load(std::memory_order_acquire)) {
        return; // 정지 상태
    }

    if (ec) {
        if (ec == boost::asio::error::operation_aborted) {
            // stop()으로 취소된 케이스
            return;
        }
        ROS_ERROR_STREAM("tcp server " << m_id << " accept error (" << ec.message() << "), retry in " << m_accept_retry_ms << "ms");
        schedule_accept_retry();
        return;
    }

    ROS_INFO_STREAM("tcp server " << m_id << " accepted: local="
                    << endpoint::to_string(m_local_endpoint)
                    << " remote=" << endpoint::to_string(v2x_interface::endpoint::to_ros(sock->remote_endpoint())));

    // per-connection 옵션 적용
    apply_socket_options(sock);

    // 외부로 소켓 소유권 전달 (strand 컨텍스트에서 호출)
    try {
        if (m_callback) m_callback(sock);
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("tcp server " << m_id << " connection callback threw: " << e.what());
    }

    // 다음 accept
    async_accept();
}

void tcp_server_t::apply_socket_options(const socket_ptr& s) noexcept
{
    if (!s) return;
    boost::system::error_code ec;

    // 지연 최소화
    s->set_option(tcp::no_delay(true), ec);
    if (ec) ROS_WARN_STREAM("tcp server " << m_id << " set TCP_NODELAY failed: " << ec.message()); ec.clear();

    // dead peer 탐지
    s->set_option(boost::asio::socket_base::keep_alive(true), ec);
    if (ec) ROS_WARN_STREAM("tcp server " << m_id << " set KEEPALIVE failed: " << ec.message()); ec.clear();

    // 필요 시 버퍼/linger 등 추가:
    // s->set_option(boost::asio::socket_base::receive_buffer_size(1<<20), ec);
    // s->set_option(boost::asio::socket_base::linger(true, 0), ec);
}

void tcp_server_t::schedule_accept_retry() noexcept
{
    if (m_stopped) return;
    m_retry_timer.expires_after(std::chrono::milliseconds(m_accept_retry_ms));
    auto self = shared_from_this();
    m_retry_timer.async_wait(
        v2x_asio::bind_exec_compat(m_strand,
            [self](const boost::system::error_code& ec) noexcept {
                if (ec || self->m_stopped) return; // cancelled
                self->async_accept();
            }
        )
    );
}

} // namespace v2x_interface
