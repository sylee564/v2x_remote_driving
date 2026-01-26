#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <tod_msgs/Status.h>
#include <tod_msgs/ControlCmd.h>
#include <tod_msgs/VehicleEnums.h>
#include <tod_msgs/joystickConfig.h>
#include <tod_msgs/ProbeVehicleData.h>

#include <tod_core/VehicleParameters.h>
#include <tod_helper/vehicle/Model.h>
#include <tod_helper/vehicle/speed_governor.hpp>

#include <map>
#include <memory>
#include <vector>
#include <string>
#include <cstdint>
#include <algorithm>
#include <functional> // std::hash

namespace tod_control {

/// tod_helper 네임스페이스 별칭
namespace VH = tod_helper::Vehicle;

class CommandCreator {
public:
  explicit CommandCreator(ros::NodeHandle& nh);
  ~CommandCreator() = default;

  void run();

private:
  // --- Callbacks ---
  void callbackJoystick(const sensor_msgs::Joy::ConstPtr& msg);
  void callbackStatus (const tod_msgs::Status& msg);
  void callbackProbe  (const tod_msgs::ProbeVehicleData::ConstPtr& msg);

  // --- Builders / Calculations ---
  void initControlMsg();
  void initOptionFlags();

  void rawLongitudinal(tod_msgs::ControlCmd& out,
                       const sensor_msgs::Joy::ConstPtr& msg, int gear);

  void calcDesiredVelocity(tod_msgs::ControlCmd& out,
                           const sensor_msgs::Joy::ConstPtr& msg, int gear);

  void calcSteeringCmd(tod_msgs::ControlCmd& out,
                       const std::vector<float>& axes);

  // --- Toggles ---
  void setGear      (tod_msgs::ControlCmd& out, const std::vector<int>& buttons, float currentVel);
  void setIndicator (tod_msgs::ControlCmd& out, const std::vector<int>& buttons);
  void setRemote    (tod_msgs::ControlCmd& out, const std::vector<int>& buttons, float currentVel);
  void setVideo     (tod_msgs::ControlCmd& out, const std::vector<int>& buttons, float currentVel);
  void setAVM       (tod_msgs::ControlCmd& out, const std::vector<int>& buttons, float currentVel);
  void setParking   (tod_msgs::ControlCmd& out, const std::vector<int>& buttons);
  void setObject    (tod_msgs::ControlCmd& out, const std::vector<int>& buttons);
  void setEmergency (tod_msgs::ControlCmd& out, const std::vector<int>& buttons);
  void setCruise    (tod_msgs::ControlCmd& out, const std::vector<int>& buttons);

  // --- Utils ---
  static inline double mapLin(double x, double in_min, double in_max, double out_min, double out_max) {
    const double t = (x - in_min) / (in_max - in_min);
    return out_min + (out_max - out_min) * std::min(1.0, std::max(0.0, t));
  }
  static inline bool inRange(std::size_t i, std::size_t n) { return i < n; }
  static inline std::uint64_t nowMs() {
    return static_cast<std::uint64_t>(ros::Time::now().toNSec() / 1000000ULL);
  }

private:
  // ROS
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_; // "~"
  ros::Subscriber sub_joy_;
  ros::Subscriber sub_status_;
  ros::Subscriber sub_probe_;
  ros::Publisher  pub_cmd_;

  // Params helpers
  std::unique_ptr<tod_core::VehicleParameters> veh_params_;

  // State
  std::map<joystick::ButtonPos, int> prev_btn_;
  tod_msgs::ControlCmd control_;

  std::uint8_t status_{tod_msgs::Status::TOD_STATUS_IDLE};
  std::uint8_t control_type_{tod_msgs::Status::CONTROL_MODE_DIRECT};

  // Governor (tod_helper에 둔 버전 사용)
  VH::Governor::SpeedGovernorParams gov_params_;
  VH::Governor::SpeedGovernor       governor_;

  // Options / params
  bool   limit_steer_rate_{false};
  bool   invert_steer_reverse_{false};     // reserved
  double max_speed_mps_{10.0};             // m/s
  double max_acc_mps2_{1.0};               // m/s^2
  double max_dec_mps2_{3.0};               // m/s^2
  double max_swa_rate_degps_{150.0};       // deg/s
  std::string operator_id_{"Kona_oper"};

  // Vehicle cache (from ProbeVehicleData)
  double       veh_speed_mps_{0.0};
  std::uint8_t veh_gear_{0};
  std::uint8_t veh_eps_status_{0};
  std::uint8_t veh_acc_status_{0};

  bool joystick_ready_{false};

  // Steering internals
  ros::Time t_prev_;
  double    swa_prev_deg_{0.0};
};

} // namespace tod_control
