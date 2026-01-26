#pragma once
#include <boost/version.hpp>
#include <boost/asio.hpp>

namespace v2x_asio {

#if BOOST_VERSION >= 107000
  using executor_t = boost::asio::io_context::executor_type;
  using strand_t   = boost::asio::strand<executor_t>;

  inline strand_t make_strand_compat(boost::asio::io_context& io) {
    return strand_t{io.get_executor()};
  }

  template <class Strand, class F>
  inline void post_compat(Strand& s, F&& f) {
    boost::asio::post(s, std::forward<F>(f));
  }

  template <class Strand, class Handler>
  inline auto bind_exec_compat(Strand& s, Handler&& h) {
    return boost::asio::bind_executor(s, std::forward<Handler>(h));
  }

#else
  // 구버전: TS executors 이전
  using strand_t = boost::asio::io_context::strand;

  inline strand_t make_strand_compat(boost::asio::io_context& io) {
    return strand_t{io};
  }

  template <class Strand, class F>
  inline void post_compat(Strand& s, F&& f) {
    s.post(std::forward<F>(f));
  }

  template <class Strand, class Handler>
  inline auto bind_exec_compat(Strand& s, Handler&& h) {
    return s.wrap(std::forward<Handler>(h));
  }
#endif

} // namespace v2x_asio
