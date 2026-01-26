#include "command_creator.hpp"
#include <cmath>
#include <algorithm>

namespace tod_control {

using joystick::ButtonPos;
using joystick::AxesPos;

CommandCreator::CommandCreator(ros::NodeHandle& nh)
: nh_(nh), pnh_("~"), governor_(gov_params_) {
  sub_joy_    = nh_.subscribe("/Operator/InputDevices/joystick", 1, &CommandCreator::callbackJoystick, this);
  sub_status_ = nh_.subscribe("/Operator/Manager/status_msg",     1, &CommandCreator::callbackStatus,  this);
  sub_probe_  = nh_.subscribe("/Operator/kona/probe_vehicle_data",1, &CommandCreator::callbackProbe,   this);
  pub_cmd_    = nh_.advertise<tod_msgs::ControlCmd>("control_cmd_data", 1);

  veh_params_ = std::make_unique<tod_core::VehicleParameters>(nh_);

  // init button states (존재하는 enum만 초기화)
  for (auto b : {
      ButtonPos::INDICATOR_LEFT, ButtonPos::INDICATOR_RIGHT, ButtonPos::EMERGENCY_SIGNAL,
      ButtonPos::INCREASE_SPEED, ButtonPos::DECREASE_SPEED,
      ButtonPos::PARK, ButtonPos::REVERSE, ButtonPos::NEUTRAL, ButtonPos::DRIVE,
      ButtonPos::REMOTE, ButtonPos::VIDEO, ButtonPos::AVM,
      ButtonPos::CRUISE, ButtonPos::PARK_MODE, ButtonPos::OBJECT
  }) {
    prev_btn_[b] = 0;
  }

  // params (private namespace)
  pnh_.param("ConstraintSteeringRate",    limit_steer_rate_,        false);
  pnh_.param("maxVelocity_mps",           max_speed_mps_,           10.0);   // m/s
  pnh_.param("maxAcceleration_mps2",      max_acc_mps2_,            1.0);    // m/s^2
  pnh_.param("maxDeceleration_mps2",      max_dec_mps2_,            3.0);    // m/s^2 (positive, 내부에서 -부호 사용)
  pnh_.param("maxSteeringRate_degps",     max_swa_rate_degps_,      150.0);  // deg/s
  pnh_.param("OperatorID",                operator_id_,             std::string("Kona_oper"));

  // governor 파라미터 로드
  pnh_.param("gov/enabled",   gov_params_.enabled,   true);
  pnh_.param("gov/v_max_mps", gov_params_.v_max_mps, max_speed_mps_);
  pnh_.param("gov/hyst_mps",  gov_params_.hyst_mps,  0.7);
  pnh_.param("gov/Kp",        gov_params_.Kp,        1.5);
  pnh_.param("gov/a_min",     gov_params_.a_min,    -max_dec_mps2_);
  pnh_.param("gov/a_max",     gov_params_.a_max,     max_acc_mps2_);
  governor_.setParams(gov_params_);

  ROS_INFO_STREAM("CommandCreator up | vmax=" << max_speed_mps_ << " m/s"
      << " | a_lim=[" << -max_dec_mps2_ << "," << max_acc_mps2_ << "] m/s^2"
      << " | steer_rate_max=" << max_swa_rate_degps_ << " deg/s"
      << " | gov: enabled=" << gov_params_.enabled
      << " v_max=" << gov_params_.v_max_mps << " hyst=" << gov_params_.hyst_mps
      << " Kp=" << gov_params_.Kp);
}

void CommandCreator::run() {
  ros::Rate r(100);
  while (ros::ok()) {
    ros::spinOnce();
    if (veh_params_ && veh_params_->vehicle_id_has_changed())
      veh_params_->load_parameters();

    // if (joystick_ready_ && status_ == tod_msgs::Status::TOD_STATUS_TELEOPERATION) {
    if (joystick_ready_) {
      control_.header.stamp = ros::Time::now();
      control_.header.frame_id = "operator_command";
      // operator_id가 uint 타입이면 문자열 해시 사용
      control_.operator_id = operator_id_;
      pub_cmd_.publish(control_);
    }
    joystick_ready_ = false;
    r.sleep();
  }
}

void CommandCreator::callbackProbe(const tod_msgs::ProbeVehicleData::ConstPtr& msg) {
  // ProbeVehicleData 정의에 맞춰 사용하세요.
  // 여기서는 velocity(km/h), gear_status 가 있다고 가정
  veh_speed_mps_ = VH::Model::kph2mps(static_cast<double>(msg->velocity));
  veh_gear_      = msg->gear_status;

  // 필요 시 EPS/ACC 상태 필드가 있으면 채우세요.
  // veh_eps_status_ = msg->vehicle_eps_Status;
  // veh_acc_status_ = msg->vehicle_acc_Status;
}

void CommandCreator::callbackStatus(const tod_msgs::Status& msg) {
  // if (status_ == tod_msgs::Status::TOD_STATUS_TELEOPERATION &&
  //     msg.tod_status != tod_msgs::Status::TOD_STATUS_TELEOPERATION) {
  //   initControlMsg();
  // }
  control_type_ = msg.operator_control_mode;
  status_ = msg.tod_status;
}

void CommandCreator::callbackJoystick(const sensor_msgs::Joy::ConstPtr& m) {
  // if (status_ != tod_msgs::Status::TOD_STATUS_TELEOPERATION) {
  //   initOptionFlags();
  //   initControlMsg();
  //   return;
  // }

  // 버튼/옵션 처리
  setRemote   (control_, m->buttons, static_cast<float>(veh_speed_mps_));
  setVideo    (control_, m->buttons, static_cast<float>(veh_speed_mps_));
  setAVM      (control_, m->buttons, static_cast<float>(veh_speed_mps_));
  setParking   (control_, m->buttons);
  setObject   (control_, m->buttons);

  if (control_.remote_flag) {
    calcSteeringCmd     (control_, m->axes);
    rawLongitudinal     (control_, m, veh_gear_);
    calcDesiredVelocity (control_, m, veh_gear_);
    setGear             (control_, m->buttons, static_cast<float>(veh_speed_mps_));
    setIndicator        (control_, m->buttons);
    setEmergency        (control_, m->buttons);
    setCruise           (control_, m->buttons);
  } else {
    initControlMsg();
  }

  joystick_ready_ = true;
}

// ----- Steering -----
void CommandCreator::calcSteeringCmd(tod_msgs::ControlCmd& out,
                                     const std::vector<float>& axes) {
  if (!inRange(AxesPos::STEERING, axes.size())) return;

  const ros::Time now = ros::Time::now();
  if (t_prev_.isZero()) t_prev_ = now;
  const double dt = std::max(1e-3, (now - t_prev_).toSec());
  t_prev_ = now;

  const double max_swa_deg = veh_params_->get_max_swa_deg();
  const double desired_deg = static_cast<double>(axes[AxesPos::STEERING]) * max_swa_deg;

  double new_deg = desired_deg;
  if (limit_steer_rate_) {
    const double delta_max = max_swa_rate_degps_ * dt;
    new_deg = std::clamp(desired_deg, swa_prev_deg_ - delta_max, swa_prev_deg_ + delta_max);
  }

  out.control.steering_angle = static_cast<float>(new_deg);

  const double rate_degps = (new_deg - swa_prev_deg_) / dt;
  out.control.steering_angle_velocity = static_cast<float>(std::fabs(rate_degps));

  // turn cancel (latch + hysteresis)
  static bool left_latched  = false;
  static bool right_latched = false;

  constexpr double ON_TH  = 70.0;
  constexpr double OFF_TH = 50.0;
  constexpr double BACK_RATE_MIN = 5.0;

  if (out.control.steering_angle >=  ON_TH) left_latched  = true;
  if (out.control.steering_angle <= -ON_TH) right_latched = true;

  if (out.indicator.data == 1) { // LEFT
    if (left_latched && out.control.steering_angle <= OFF_TH && rate_degps < -BACK_RATE_MIN) {
      out.indicator.data = 0;
      left_latched = false;
    }
  } else if (out.indicator.data == 2) { // RIGHT
    if (right_latched && out.control.steering_angle >= -OFF_TH && rate_degps > +BACK_RATE_MIN) {
      out.indicator.data = 0;
      right_latched = false;
    }
  } else {
    left_latched = right_latched = false;
  }

  swa_prev_deg_ = new_deg;
}

// ----- Longitudinal (raw pedals) -----
void CommandCreator::rawLongitudinal(tod_msgs::ControlCmd& out,
                                     const sensor_msgs::Joy::ConstPtr& m, int gear) {
  if (gear == eGearPosition::GEARPOSITION_PARK || gear == eGearPosition::GEARPOSITION_NEUTRAL) {
    out.control.throttle = 0.0f;
    out.control.brake    = 0.0f;
    out.acceleration     = 0.0f;
    out.velocity         = 0.0f;
    return;
  }

  bool separateBrakeAxis = true;
  pnh_.param("InputDeviceHasSeparateBrakingAxis", separateBrakeAxis, true);

  auto map01 = [](double x, double lo, double hi){
    const double t = (x - lo) / (hi - lo);
    const double s = std::min(1.0, std::max(0.0, t));
    return static_cast<float>(100.0 * s);
  };

  if (!separateBrakeAxis) {
    if (!inRange(AxesPos::THROTTLE, m->axes.size())) return;
    const double thr = m->axes[AxesPos::THROTTLE];
    if (thr > 0.0) {
      out.control.throttle = (veh_speed_mps_ >= gov_params_.v_max_mps) ? 0.0f : map01(thr, 0.0, 1.0);
      out.control.brake    = 0.0f;
    } else {
      out.control.throttle = 0.0f;
      out.control.brake    = map01(thr, 0.0, -1.0);
    }
  } else {
    if (!inRange(AxesPos::THROTTLE, m->axes.size()) ||
        !inRange(AxesPos::BRAKE,    m->axes.size())) return;

    if (veh_speed_mps_ >= gov_params_.v_max_mps) {
      out.control.throttle = 0.0f;
      out.control.brake    = map01(m->axes[AxesPos::BRAKE], -1.0, 1.0);
    } else {
      out.control.throttle = map01(m->axes[AxesPos::THROTTLE], -1.0, 1.0);
      out.control.brake    = map01(m->axes[AxesPos::BRAKE],    -1.0, 1.0);
    }
  }
}

// ----- Desired velocity/acc (with speed governor) -----
void CommandCreator::calcDesiredVelocity(tod_msgs::ControlCmd& out,
                                         const sensor_msgs::Joy::ConstPtr& m, int gear) {
  if (gear == eGearPosition::GEARPOSITION_PARK || gear == eGearPosition::GEARPOSITION_NEUTRAL) {
    out.velocity     = 0.0f; // m/s
    out.acceleration = 0.0f; // m/s^2
    return;
  }

  static ros::Time t0 = ros::Time::now();
  const ros::Time now = ros::Time::now();
  const double dt = std::max(1e-3, (now - t0).toSec());
  t0 = now;

  bool separateBrakeAxis = true;
  pnh_.param("InputDeviceHasSeparateBrakingAxis", separateBrakeAxis, true);

  double change = 0.0;
  if (!separateBrakeAxis) {
    if (!inRange(AxesPos::THROTTLE, m->axes.size())) return;
    change = m->axes[AxesPos::THROTTLE];
  } else {
    if (!inRange(AxesPos::THROTTLE, m->axes.size()) ||
        !inRange(AxesPos::BRAKE,    m->axes.size())) return;
    change = (m->axes[AxesPos::THROTTLE] - m->axes[AxesPos::BRAKE]) * 0.5;
  }

  // deadzone → accel/decel
  constexpr double dz = 0.01;
  double a_cmd = 0.0; // m/s^2
  if (change > 0.0) {
    a_cmd = max_acc_mps2_ * (std::max(change, dz) - dz);
  } else {
    a_cmd = -max_dec_mps2_ * (std::max(-change, dz) - dz); // 음수(감속)
  }

  // 1) P-기반 소프트 캡 + 히스테리시스
  a_cmd = governor_.apply(a_cmd, veh_speed_mps_);

  // 2) 하드 캡: 이번 주기 내 v_max 넘기지 않도록 양(+) 가속 제한
  if (a_cmd > 0.0) {
    const double a_dt_cap = (gov_params_.v_max_mps - veh_speed_mps_) / dt;
    a_cmd = std::min(a_cmd, a_dt_cap);
    if (veh_speed_mps_ >= gov_params_.v_max_mps - 1e-3) a_cmd = 0.0;
  }

  // 3) 로컬 참조 속도 적분 + 클램프
  double v_ref = static_cast<double>(out.velocity) + a_cmd * dt; // m/s
  v_ref = std::clamp(v_ref, 0.0, gov_params_.v_max_mps);

  // 4) 버튼으로 ±1 km/h
  if (inRange(ButtonPos::INCREASE_SPEED, m->buttons.size())) {
    static int prev = 0;
    if (m->buttons[ButtonPos::INCREASE_SPEED] == 1 && prev == 0)
      v_ref = std::clamp(v_ref + (1.0/3.6), 0.0, gov_params_.v_max_mps);
    prev = m->buttons[ButtonPos::INCREASE_SPEED];
  }
  if (inRange(ButtonPos::DECREASE_SPEED, m->buttons.size())) {
    static int prev = 0;
    if (m->buttons[ButtonPos::DECREASE_SPEED] == 1 && prev == 0)
      v_ref = std::clamp(v_ref - (1.0/3.6), 0.0, gov_params_.v_max_mps);
    prev = m->buttons[ButtonPos::DECREASE_SPEED];
  }

  // 5) 최종 출력
  out.velocity     = static_cast<float>(v_ref); // m/s
  out.acceleration = static_cast<float>(a_cmd); // m/s^2
}

// ----- Toggles -----
void CommandCreator::setGear(tod_msgs::ControlCmd& out, const std::vector<int>& btn, float curV) {
  if (curV >= 0.01f) return;

  auto tap = [&](ButtonPos b, uint8_t v){
    if (!inRange(b, btn.size())) return;
    static std::map<ButtonPos,int> prev;
    int &pv = prev[b];
    if (btn[b] == 1 && pv == 0) out.shift.data = v;
    pv = btn[b];
  };
  tap(ButtonPos::PARK,    eGearPosition::GEARPOSITION_PARK);
  tap(ButtonPos::REVERSE, eGearPosition::GEARPOSITION_REVERSE);
  tap(ButtonPos::NEUTRAL, eGearPosition::GEARPOSITION_NEUTRAL);
  tap(ButtonPos::DRIVE,   eGearPosition::GEARPOSITION_DRIVE);
}

void CommandCreator::setIndicator(tod_msgs::ControlCmd& out, const std::vector<int>& btn) {
  auto toggle = [&](ButtonPos b, uint8_t val){
    if (!inRange(b, btn.size())) return;
    static std::map<ButtonPos,int> prev;
    int &pv = prev[b];
    if (btn[b] == 1 && pv == 0) {
      if (out.indicator.data == val) out.indicator.data = 0;
      else if (out.indicator.data != 3) out.indicator.data = val;
    }
    pv = btn[b];
  };
  toggle(ButtonPos::INDICATOR_LEFT,  1);
  toggle(ButtonPos::INDICATOR_RIGHT, 2);

  if (inRange(ButtonPos::EMERGENCY_SIGNAL, btn.size())) {
    static int prev = 0;
    if (btn[ButtonPos::EMERGENCY_SIGNAL] == 1 && prev == 0)
      out.indicator.data = (out.indicator.data == 3) ? 0 : 3;
    prev = btn[ButtonPos::EMERGENCY_SIGNAL];
  }
}

void CommandCreator::setRemote(tod_msgs::ControlCmd& out, const std::vector<int>& btn, float curV) {
  if (curV >= 0.01f) return;
  // 차량 상태 체크를 쓰고 싶으면 ProbeVehicleData의 필드명에 맞춰 해제
  // if (veh_eps_status_ == 0 || veh_acc_status_ == 0) return;

  if (inRange(ButtonPos::REMOTE, btn.size())) {
    static int prev = 0;
    if (btn[ButtonPos::REMOTE] == 1 && prev == 0)
      out.remote_flag = !out.remote_flag;
    prev = btn[ButtonPos::REMOTE];
  }
  if (!out.remote_flag) initControlMsg();
}

void CommandCreator::setVideo(tod_msgs::ControlCmd& out, const std::vector<int>& btn, float curV) {
  if (curV >= 0.01f) return;
  if (inRange(ButtonPos::VIDEO, btn.size())) {
    static int prev = 0;
    if (btn[ButtonPos::VIDEO] == 1 && prev == 0)
      out.stream_flag = !out.stream_flag;
    prev = btn[ButtonPos::VIDEO];
  }
}

void CommandCreator::setAVM(tod_msgs::ControlCmd& out, const std::vector<int>& btn, float curV) {
  if (curV >= 0.01f) return;
  if (inRange(ButtonPos::AVM, btn.size())) {
    static int prev = 0;
    if (btn[ButtonPos::AVM] == 1 && prev == 0)
      out.avm_flag = !out.avm_flag;
    prev = btn[ButtonPos::AVM];
  }
}

void CommandCreator::setParking(tod_msgs::ControlCmd& out, const std::vector<int>& btn) {
  if (!out.stream_flag) return;
  if (inRange(ButtonPos::PARK_MODE, btn.size())) {
    static int prev = 0;
    if (btn[ButtonPos::PARK_MODE] == 1 && prev == 0)
      out.parking_flag = !out.parking_flag;
    prev = btn[ButtonPos::PARK_MODE];
  }
  if (out.parking_flag) out.avm_flag = true;
  else out.avm_flag = false;
}

void CommandCreator::setObject(tod_msgs::ControlCmd& out, const std::vector<int>& btn) {
  if (!out.stream_flag) return;
  if (inRange(ButtonPos::OBJECT, btn.size())) {
    static int prev = 0;
    if (btn[ButtonPos::OBJECT] == 1 && prev == 0)
      out.object_flag = !out.object_flag;
    prev = btn[ButtonPos::OBJECT];
  }
}

void CommandCreator::setEmergency(tod_msgs::ControlCmd& out, const std::vector<int>& btn) {
  if (inRange(ButtonPos::AEB, btn.size())) {
    static int prev = 0;
    if (btn[ButtonPos::AEB] == 1 && prev == 0)
      out.aeb_flag = !out.aeb_flag;
    prev = btn[ButtonPos::AEB];
  }
}

void CommandCreator::setCruise(tod_msgs::ControlCmd& out, const std::vector<int>& btn) {
  if (inRange(ButtonPos::CRUISE, btn.size())) {
    static int prev = 0;
    if (btn[ButtonPos::CRUISE] == 1 && prev == 0)
      out.cruise_flag = !out.cruise_flag;
    prev = btn[ButtonPos::CRUISE];
  }
}

// ----- Init -----
void CommandCreator::initControlMsg() {
  control_.control.throttle = 0.0f;
  control_.control.brake = 0.0f;
  control_.control.steering_angle = 0.0f; // deg
  control_.velocity = 0.0f;
  control_.acceleration = 0.0f;
  control_.indicator.data = eIndicator::INDICATOR_OFF;
  control_.shift.data = eGearPosition::GEARPOSITION_PARK;
  control_.aeb_flag = false;
  control_.cruise_flag = false;
}

void CommandCreator::initOptionFlags() {
  control_.remote_flag  = false;
  control_.stream_flag  = false;
  control_.avm_flag     = false;
  control_.object_flag  = false;
  control_.parking_flag = false;
}

} // namespace tod_control
