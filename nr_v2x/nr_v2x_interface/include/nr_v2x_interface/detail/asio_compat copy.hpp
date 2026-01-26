#pragma once
#include <boost/version.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/strand.hpp>
#include <boost/asio/post.hpp>
#include <boost/asio/dispatch.hpp>
#include <boost/asio/bind_executor.hpp>
#include <utility>

namespace v2x_asio {

// executor / strand 타입 정의
#if BOOST_VERSION >= 107400
  using executor_t = boost::asio::any_io_executor;
  using strand_t   = boost::asio::strand<executor_t>;
  inline strand_t make_strand_compat(boost::asio::io_context& io) {
    return boost::asio::make_strand(io.get_executor());
  }
  template <class Handler>
  auto bind_exec_compat(strand_t& s, Handler&& h) {
    return boost::asio::bind_executor(s, std::forward<Handler>(h));
  }
#else
  using executor_t = boost::asio::io_context::executor_type;
  using strand_t   = boost::asio::io_context::strand;
  inline strand_t make_strand_compat(boost::asio::io_context& io) { return strand_t(io); }
  template <class Handler>
  auto bind_exec_compat(strand_t& s, Handler&& h) {
    return s.wrap(std::forward<Handler>(h));
  }
#endif

// 공통 헬퍼: handler를 strand에 바인딩하여 post/dispatch
template <class F>
inline void post_compat(strand_t& s, F&& f) {
  // strand를 executor로 직접 전달
  boost::asio::post(s, bind_exec_compat(s, std::forward<F>(f)));
}
template <class F>
inline void dispatch_compat(strand_t& s, F&& f) {
  // strand를 executor로 직접 전달
  boost::asio::dispatch(s, bind_exec_compat(s, std::forward<F>(f)));
}

// 가독성용 래핑(기존 코드 호환)
template <class Handler>
inline auto wrap_handler(strand_t& s, Handler&& h) {
  return bind_exec_compat(s, std::forward<Handler>(h));
}

} // namespace v2x_asio
