#pragma once

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <tf/transform_datatypes.h>

#include <mutex>
#include <string>
#include <cstdint>
#include <cmath>

// tod_msgs
#include "tod_msgs/Status.h"
#include "tod_msgs/ControlCmd.h"
#include "tod_msgs/ProbeVehicleData.h"

// Vehicle reports (decoded by your CAN bridge)
#include "kona_driver_msgs/SteerReport.h"
#include "kona_driver_msgs/AccReport.h"
#include "kona_driver_msgs/BrakeReport.h"
#include "kona_driver_msgs/WheelSpeedReport.h"
#include "kona_driver_msgs/GearReport.h"

// Outputs toward CAN encoders
#include "kona_driver_msgs/PrimaryCommand.h"
#include "kona_driver_msgs/SecondCommand.h"


#include <tod_helper/vehicle/speed_governor.hpp>
#include <tod_helper/vehicle/Model.h>

namespace tod_kona_bridge {
namespace VH = tod_helper::Vehicle;
// -----------------------------
// Data structures
// -----------------------------
struct DirectControlOutput {
  int   target_steer_angle{0};     // deg
  uint8_t   steering_angle_velocity{0};     // deg/s
  float target_acceleration{0.0f}; // m/s^2, clamp [-3.00, +1.00] floored(0.01)
  std::uint8_t target_indicator{0}; // 0:none,1:left,2:right,3:hazard
  std::uint8_t target_gear{0};      // 0..15
  bool remote_flag{false};
  bool aeb_flag{false};
};

struct ProbeVehicleDataCache {
  // identity
  std::string vehicle_name;
  std::string vehicle_id;
  std::uint16_t vehicle_type{0};

  // last seen reports
  kona_driver_msgs::SteerReport      steer{};
  kona_driver_msgs::AccReport        acc{};
  kona_driver_msgs::BrakeReport      brake{};
  kona_driver_msgs::WheelSpeedReport wheel{};
  kona_driver_msgs::GearReport       gear{};

  // gnss/heading
  double lat{0.0};
  double lon{0.0};
  float  alt{0.0f};
  float  heading_deg{0.0f}; // 0..360
  std::uint8_t gnss_status{0}; // 0: no-fix/unknown, 1: fix, 2: dgps/rtk 등 필요시 세분화

  // misc
  std::uint16_t vehicle_mile{0};     // optional
  std::uint8_t  vehicle_mode_status{0};

  // std::uint8_t acc_control_status{0};
  // std::uint8_t eps_control_status{0};

};

// ---------------------------------
// Main node combining both pipelines
// ---------------------------------
class KonaControlProbeNode {
public:
  KonaControlProbeNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
  // Callbacks
  void onControlCmd(const tod_msgs::ControlCmd::ConstPtr& m);

  void onSteer(const kona_driver_msgs::SteerReport::ConstPtr& m);
  void onAcc(const kona_driver_msgs::AccReport::ConstPtr& m);
  void onBrake(const kona_driver_msgs::BrakeReport::ConstPtr& m);
  void onWheel(const kona_driver_msgs::WheelSpeedReport::ConstPtr& m);
  void onGear(const kona_driver_msgs::GearReport::ConstPtr& m);

  void onFix(const sensor_msgs::NavSatFix::ConstPtr& m);
  void onHeading(const geometry_msgs::QuaternionStamped::ConstPtr& m);

  void onTodStatus(const tod_msgs::Status::ConstPtr& m);


  // Periodic publish
  void onTimer(const ros::TimerEvent&);

  // Builders
  void buildPrimaryCommand(const DirectControlOutput& in, kona_driver_msgs::PrimaryCommand& out);
  void buildSecondCommand (const DirectControlOutput& in, const ProbeVehicleDataCache& c,
                           kona_driver_msgs::SecondCommand& out);
  void buildProbeVehicle  (const ProbeVehicleDataCache& cache, tod_msgs::ProbeVehicleData& out);

  static inline float clampAccelFloor2(float a);

private:
  ros::NodeHandle nh_, pnh_;

  // pubs
  ros::Publisher pub_primary_;
  ros::Publisher pub_second_;
  ros::Publisher pub_pvd_;

  // subs
  ros::Subscriber sub_tod_status_;
  ros::Subscriber sub_cmd_;
  ros::Subscriber sub_steer_;
  ros::Subscriber sub_acc_;
  ros::Subscriber sub_brake_;
  ros::Subscriber sub_wheel_;
  ros::Subscriber sub_gear_;
  ros::Subscriber sub_fix_;
  ros::Subscriber sub_heading_;

  // timer
  ros::Timer timer_;

  // params
  double loop_rate_hz_{100.0};
  bool speed_is_kmh_{true};

  // topics
  std::string topic_cmd_;
  std::string topic_primary_;
  std::string topic_second_;
  std::string topic_steer_;
  std::string topic_acc_;
  std::string topic_brake_;
  std::string topic_wheel_;
  std::string topic_gear_;
  std::string topic_fix_;
  std::string topic_heading_;
  std::string topic_pvd_;

  // ids
  std::string  vehicle_name_;
  std::string  vehicle_id_;
  std::uint16_t vehicle_type_{0};

  uint8_t tod_status_{0};

  // states
  std::mutex mtx_;
  DirectControlOutput control_;
  ProbeVehicleDataCache cache_;

  // checksums (rolling)
  std::uint8_t checksum_157_{0};
  std::uint8_t checksum_156_{0};

  // speed governor (2차 보호막)
  VH::Governor::SpeedGovernor       governor_;
  VH::Governor::SpeedGovernorParams gov_param_;

  uint8_t eps_speed_{150};

  // once log
  bool printed_first_data_{false};
};

} // namespace tod_kona_bridge
