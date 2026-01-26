#pragma once
#include <chrono>
#include <atomic>

// 간단 경량 레이트 리미터: min_interval_ms 간격 보장
class QRateLimiter {
public:
  explicit QRateLimiter(int min_interval_ms = 50)
  : min_interval_{min_interval_ms},
    last_{std::chrono::steady_clock::now() - std::chrono::milliseconds(min_interval_ms)} {}

  // 호출 가능하면 true 반환하고 내부 타임스탬프 갱신
  bool allow() {
    using namespace std::chrono;
    const auto now = steady_clock::now();
    const auto due = last_.load(std::memory_order_relaxed) + milliseconds(min_interval_);
    if (now >= due) {
      last_.store(now, std::memory_order_relaxed);
      return true;
    }
    return false;
  }

  void set_min_interval(int ms) { min_interval_ = ms; }
  int  min_interval() const     { return min_interval_; }

private:
  int min_interval_;
  std::atomic<std::chrono::steady_clock::time_point> last_;
};
