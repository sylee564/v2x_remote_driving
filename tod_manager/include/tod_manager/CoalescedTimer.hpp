#pragma once
#include <QObject>
#include <QTimer>
#include <functional>

class CoalescedTimer : public QObject {
public:
  explicit CoalescedTimer(
      int interval_ms,
      std::function<void()> callback,
      QObject* parent = nullptr)
    : QObject(parent),
      cb_(std::move(callback))
  {
    timer_.setInterval(interval_ms);
    timer_.setSingleShot(true);
    // timer 만료 시 pending 해제 후 콜백 1회 호출
    QObject::connect(&timer_, &QTimer::timeout, this, [this]() {
      pending_ = false;
      if (cb_) cb_();
    });
  }

  // interval 동안 여러 번 불러도 1회만 콜백 예약
  inline void poke() {
    if (!pending_) {
      pending_ = true;
      timer_.start();
    }
    // 이미 pending이면 무시(코얼레싱)
  }

  inline void setInterval(int ms) { timer_.setInterval(ms); }
  inline bool isPending() const   { return pending_; }

private:
  QTimer timer_;
  std::function<void()> cb_;
  bool pending_{false};
};
