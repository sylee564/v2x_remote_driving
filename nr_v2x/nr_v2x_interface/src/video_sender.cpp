#include "nr_v2x_interface/video_sender.hpp"
#include <algorithm>
#include <cstring>
#include <arpa/inet.h> // htonl/htons

// RAW-MUX 헤더는 여기서 선언(또는 공용 헤더에서 include)
namespace v2x { namespace proto { namespace rawmux {
#pragma pack(push, 1)
struct AppMuxHdr {
  uint8_t  kind;       // 2 = VIDEO
  uint8_t  stream;     // 0,1,2...
  uint8_t  flags;      // bit0: keyframe
  uint8_t  _pad;
  uint32_t frame_seq;  // BE
  uint16_t frag_idx;   // BE
  uint16_t frag_cnt;   // BE
  uint32_t frame_len;  // BE
};
#pragma pack(pop)
}}}

namespace v2x_interface {

using v2x::proto::rawmux::AppMuxHdr;

VideoSender::VideoSender(boost::asio::io_context& io,
                         std::shared_ptr<std::vector<uint8_t>> frame,
                         int stream_id, bool keyframe, uint32_t frame_seq,
                         uint32_t frag_bytes, uint32_t pacing_us,
                         BuildWireFn build_fn, SendWireFn send_fn, NextSeqFn next_seq_fn)
  : io_(io),
    frame_(std::move(frame)),
    data_(frame_ ? frame_->data() : nullptr),
    n_(frame_ ? frame_->size() : 0),
    stream_id_(stream_id),
    keyframe_(keyframe),
    frame_seq_(frame_seq),
    frag_bytes_(std::max<uint32_t>(200, frag_bytes)),
    frag_cnt_(static_cast<uint16_t>((n_ + std::max<uint32_t>(200, frag_bytes) - 1) / std::max<uint32_t>(200, frag_bytes))),
    pacing_us_(pacing_us),
    timer_(std::make_shared<boost::asio::steady_timer>(io_)),
    build_fn_(std::move(build_fn)),
    send_fn_(std::move(send_fn)),
    next_seq_fn_(std::move(next_seq_fn))
{}

void VideoSender::start() {
  if (!frame_ || n_ == 0 || !build_fn_ || !send_fn_ || !next_seq_fn_) return;
  // 항상 io_context 스레드에서 시작
  boost::asio::post(timer_->get_executor(),
                    [self = shared_from_this()] { self->step({}); });
}

void VideoSender::step(const boost::system::error_code& ec) {
  if (ec) return;
  if (off_ >= n_) return;

  const size_t take = std::min<size_t>(frag_bytes_, n_ - off_);
  make_mux_and_emit(data_ + off_, take, idx_);
  off_ += take;
  ++idx_;

  if (off_ < n_) {
    if (pacing_us_ == 0) {
      boost::asio::post(timer_->get_executor(),
                        [self = shared_from_this()] { self->step({}); });
    } else {
      timer_->expires_after(std::chrono::microseconds(pacing_us_));
      timer_->async_wait([self = shared_from_this()](const boost::system::error_code& e){
        self->step(e);
      });
    }
  }
}

void VideoSender::make_mux_and_emit(const uint8_t* p, size_t m, uint16_t frag_idx) {
  AppMuxHdr mh{};
  mh.kind      = 2;
  mh.stream    = static_cast<uint8_t>(stream_id_);
  mh.flags     = keyframe_ ? 0x01 : 0x00;
  mh.frame_seq = htonl(frame_seq_);
  mh.frag_idx  = htons(frag_idx);
  mh.frag_cnt  = htons(frag_cnt_ == 0 ? 1 : frag_cnt_);
  mh.frame_len = htonl(static_cast<uint32_t>(n_));

  std::vector<uint8_t> mux(sizeof(AppMuxHdr) + m);
  std::memcpy(mux.data(), &mh, sizeof(mh));
  std::memcpy(mux.data() + sizeof(mh), p, m);

  //   const uint16_t seq = next_seq_fn_();
  const uint16_t seq = next_seq_fn_();

  // 조각별 wire 구성 → 전송
  std::vector<uint8_t> wire = build_fn_(mux.data(), mux.size(), frag_idx, frag_cnt_, seq);
  send_fn_(wire, seq);
}

} // namespace v2x_interface
