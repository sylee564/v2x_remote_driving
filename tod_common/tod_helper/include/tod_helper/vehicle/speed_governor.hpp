#pragma once
#include <algorithm>

namespace tod_helper::Vehicle::Governor {

struct SpeedGovernorParams {
  double v_max_mps { 16.7 }; // [m/s] ≈ 60 km/h
  double hyst_mps  { 0.7  }; // [m/s] 히스테리시스 폭
  double Kp        { 1.5  }; // [1/s] 전이대에서 양(+) 가속 상한에 쓰는 P 계수
  double a_min     { -3.0 }; // [m/s^2] 차량/DBC 물리 한계
  double a_max     { +1.0 }; // [m/s^2]
  bool   enabled   { true  };
};

class SpeedGovernor {
public:
  explicit SpeedGovernor(const SpeedGovernorParams& p = {}) : p_(p) {}
  void setParams(const SpeedGovernorParams& p) { p_ = p; }
  const SpeedGovernorParams& params() const { return p_; }

  // desired_accel: 상위(조이스틱 등)에서 만든 가속도 [m/s^2]
  // vehicle_speed_mps: 현재 속도 [m/s]
  double apply(double desired_accel, double vehicle_speed_mps) const {
    double a = clamp(desired_accel, p_.a_min, p_.a_max);
    
    if (!p_.enabled) return a;

    const double v_up = p_.v_max_mps - 0.5 * p_.hyst_mps;
    const double v_lo = p_.v_max_mps - 1.0 * p_.hyst_mps;

    if (vehicle_speed_mps >= v_up) {
      a = std::min(a, 0.0);               // 상한 근처: 양(+) 가속 금지
    } else if (vehicle_speed_mps > v_lo) {
      const double a_cap = p_.Kp * (p_.v_max_mps - vehicle_speed_mps); // 0..+
      a = std::min(a, a_cap);            // 전이대: 부드럽게 캡핑
    }
    return clamp(a, p_.a_min, p_.a_max);
  }

  static inline double kmh2mps(double kmh) { return kmh * (1000.0/3600.0); }

private:
  static inline double clamp(double x, double lo, double hi) {
    return std::max(lo, std::min(hi, x));
  }
  SpeedGovernorParams p_;
};

} // namespace tod_kona_bridge
