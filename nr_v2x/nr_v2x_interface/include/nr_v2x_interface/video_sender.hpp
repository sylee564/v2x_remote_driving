#pragma once
#include <cstdint>
#include <vector>
#include <memory>
#include <functional>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/post.hpp>

namespace v2x_interface {

class VideoSender : public std::enable_shared_from_this<VideoSender> {
public:
  // mux 바이트열 → 최종 전송 바이트열로 감싸는 빌더
  using BuildWireFn = std::function<std::vector<uint8_t>(
    const uint8_t* mux_bytes, size_t mux_len,
    uint16_t frag_idx, uint16_t frag_cnt,
    uint16_t seq_for_header)>;

  // 실제 소켓 송신 + 메트릭 기록
  using SendWireFn = std::function<void(const std::vector<uint8_t>& wire,
                                        uint16_t seq_for_metrics)>;

  // 외피 시퀀스(영상 TLVC 헤더 seq 등) 생성기
  using NextSeqFn  = std::function<uint16_t()>;

  VideoSender(boost::asio::io_context& io,
              std::shared_ptr<std::vector<uint8_t>> frame,
              int stream_id, bool keyframe, uint32_t frame_seq,
              uint32_t frag_bytes, uint32_t pacing_us,
              BuildWireFn build_fn, SendWireFn send_fn, NextSeqFn next_seq_fn);

  // 비동기 전송 시작
  void start();

private:
  void step(const boost::system::error_code& ec);
  void make_mux_and_emit(const uint8_t* p, size_t m, uint16_t frag_idx);

private:
  boost::asio::io_context& io_;
  std::shared_ptr<std::vector<uint8_t>> frame_;
  const uint8_t* data_{nullptr};
  size_t n_{0};

  const int stream_id_{0};
  const bool keyframe_{false};
  const uint32_t frame_seq_{0};
  const uint32_t frag_bytes_{0};
  const uint16_t frag_cnt_{0};
  const uint32_t pacing_us_{0};

  std::shared_ptr<boost::asio::steady_timer> timer_;
  size_t off_{0};
  uint16_t idx_{0};

  BuildWireFn build_fn_;
  SendWireFn  send_fn_;
  NextSeqFn   next_seq_fn_;
};

} // namespace v2x_interface
