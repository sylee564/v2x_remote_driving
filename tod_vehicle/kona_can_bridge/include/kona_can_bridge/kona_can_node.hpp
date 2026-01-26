#pragma once

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <std_msgs/Header.h>
#include <mutex>
#include <array>
#include <string>

// Kona driver msgs (사용자 패키지)
#include "kona_driver_msgs/SteerReport.h"
#include "kona_driver_msgs/AccReport.h"
#include "kona_driver_msgs/WheelSpeedReport.h"
#include "kona_driver_msgs/BrakeReport.h"
#include "kona_driver_msgs/GearReport.h"
#include "kona_driver_msgs/PrimaryCommand.h"
#include "kona_driver_msgs/SecondCommand.h"

#include "kona_can_bridge/reports_decode.hpp"
#include "kona_can_bridge/commands_encode.hpp"

namespace kona_can {

class KonaCanNode {
public:
  KonaCanNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
  // 수신 콜백 (CAN → 디코딩 → Report 퍼블리시)
  void onCanFrame(const can_msgs::Frame::ConstPtr& msg);

  // 명령 콜백 (사용자 토픽 → 최신 상태 저장)
  void onPrimaryCmd(const kona_driver_msgs::PrimaryCommand::ConstPtr& m);
  void onSecondCmd(const kona_driver_msgs::SecondCommand::ConstPtr& m);

  // 주기 송신 타이머 (최신 상태 → 인코딩 → CAN 퍼블리시)
  void onTxTimer(const ros::TimerEvent&);

private:
  ros::NodeHandle nh_, pnh_;
  std::string rx_topic_, tx_topic_, out_ns_;
  double tx_rate_hz_{50.0};

  // pubs (디코딩 결과)
  ros::Publisher pub_steer_, pub_gear_, pub_acc_, pub_brake_, pub_wheel_;
  // pub (CAN 송신)
  ros::Publisher pub_can_tx_;
  // subs
  ros::Subscriber sub_can_rx_, sub_primary_cmd_, sub_second_cmd_;
  // timer
  ros::Timer tx_timer_;

  kona_driver_msgs::PrimaryCommandConstPtr last_primary_;
  kona_driver_msgs::SecondCommandConstPtr last_second_;

  std::mutex mtx_primary_, mtx_second_;
};

} // namespace kona_can
