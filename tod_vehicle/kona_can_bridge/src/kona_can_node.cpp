#include "kona_can_bridge/kona_can_node.hpp"

namespace kona_can {

KonaCanNode::KonaCanNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
: nh_(nh), pnh_(pnh)
{
  // 파라미터
  rx_topic_   = pnh_.param<std::string>("can_rx_topic", "/rx");
  tx_topic_   = pnh_.param<std::string>("can_tx_topic", "/tx");
  out_ns_     = pnh_.param<std::string>("out_ns",        "kona");
  tx_rate_hz_ = pnh_.param("tx_rate_hz", 100.0);

  // 퍼블리셔 (디코딩 결과)
  pub_steer_ = nh_.advertise<kona_driver_msgs::SteerReport>("steer_report", 10);
  pub_gear_  = nh_.advertise<kona_driver_msgs::GearReport>("gear_report", 10);
  pub_acc_   = nh_.advertise<kona_driver_msgs::AccReport>("acc_report", 10);
  pub_brake_ = nh_.advertise<kona_driver_msgs::BrakeReport>("brake_report", 10);
  pub_wheel_ = nh_.advertise<kona_driver_msgs::WheelSpeedReport>("wheel_report", 10);

  // 퍼블리셔 (CAN 송신)
  pub_can_tx_ = nh_.advertise<can_msgs::Frame>(tx_topic_, 20);

  // 서브스크립션 (CAN 수신)
  sub_can_rx_ = nh_.subscribe(rx_topic_, 50, &KonaCanNode::onCanFrame, this);

  // 명령 토픽
  sub_primary_cmd_ = nh_.subscribe("primary_command", 10, &KonaCanNode::onPrimaryCmd, this);
  sub_second_cmd_  = nh_.subscribe("second_command",  10, &KonaCanNode::onSecondCmd,  this);

  // 주기 송신 타이머
  tx_timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, tx_rate_hz_)),
                              &KonaCanNode::onTxTimer, this);

  ROS_INFO_STREAM("kona_can_node started. rx=" << rx_topic_ << " tx=" << tx_topic_
                  << " rate=" << tx_rate_hz_);
}

// ====== CAN 수신 → 디코딩 → 퍼블리시 ======
void KonaCanNode::onCanFrame(const can_msgs::Frame::ConstPtr& msg) {
  if (!msg) return;

  // 공통 헤더(필요 시 사용)
  std_msgs::Header header;
  header.frame_id = out_ns_;
  header.stamp    = msg->header.stamp;

  const uint32_t id = msg->id;
  std::array<std::uint8_t,8> p{};
  for (int i=0;i<8;i++) p[i] = msg->data[i];

  // 0x710 Steering
  if (id == can_dec::SteeringReport710::ID) {
    can_dec::SteeringReport710 r; r.parse(p);
    kona_driver_msgs::SteerReport out;
    out.eps_control_board_status = r.eps_control_board;
    out.eps_en_status            = r.eps_en;
    out.eps_control_status       = r.eps_control_status;
    out.override_status          = r.override_status;
    out.steering_angle           = r.steering_angle_deg;
    out.str_drv_tq               = r.steer_drv_tq_Nm;
    out.str_out_tq               = r.steer_out_tq_Nm;
    out.eps_alive_cnt            = r.eps_alive_cnt;
    pub_steer_.publish(out);
    return;
  }

  // 0x720 Gear
  if (id == can_dec::GearReport720::ID) {
    can_dec::GearReport720 r; r.parse(p);
    kona_driver_msgs::GearReport out;
    out.gear_current        = r.gear_cur;
    out.gear_cmd            = r.gear_cmd;
    out.gear_control_status = r.gear_control_status;
    pub_gear_.publish(out);
    return;
  }

  // 0x711 ACC
  if (id == can_dec::AccReport711::ID) {
    can_dec::AccReport711 r; r.parse(p);
    kona_driver_msgs::AccReport out;
    out.acc_err                   = r.acc_err;
    out.acc_veh_err               = r.acc_veh_err;
    out.acc_user_can_err          = r.acc_user_can_err;
    out.acc_control_board_status  = r.acc_control_board;
    out.acc_en_status             = r.acc_en_status;
    out.acc_control_status        = r.acc_control_status;
    out.vehicle_speed             = r.vehicle_speed;
    out.long_accel                = r.long_accel_mps2;
    out.acc_alive_cnt             = r.acc_alive_cnt;
    pub_acc_.publish(out);
    return;
  }

  // 0x713 Brake
  if (id == can_dec::BrakeReport713::ID) {
    can_dec::BrakeReport713 r; r.parse(p);
    kona_driver_msgs::BrakeReport out;
    out.lat_accel     = r.lat_accel_mps2;
    out.yaw_rate      = r.yaw_rate_dps;
    out.brk_cylinder  = r.brake_cylinder_kpa;
    pub_brake_.publish(out);
    return;
  }

  // 0x712 Wheel Speed
  if (id == can_dec::WheelReport712::ID) {
    can_dec::WheelReport712 r; r.parse(p);
    kona_driver_msgs::WheelSpeedReport out;
    out.wheel_spd_fl = r.wheel_spd_fl;
    out.wheel_spd_fr = r.wheel_spd_fr;
    out.wheel_spd_rl = r.wheel_spd_rl;
    out.wheel_spd_rr = r.wheel_spd_rr;
    pub_wheel_.publish(out);
    return;
  }
}

// ====== 명령 콜백 ======
void KonaCanNode::onPrimaryCmd(const kona_driver_msgs::PrimaryCommand::ConstPtr& m) {
  if (!m) return;
  std::lock_guard<std::mutex> lk(mtx_primary_);
  last_primary_ = m;

}

void KonaCanNode::onSecondCmd(const kona_driver_msgs::SecondCommand::ConstPtr& m) {
  if (!m) return;
  std::lock_guard<std::mutex> lk(mtx_second_);
  last_second_ = m;
}

// ====== 주기 송신 ======
void KonaCanNode::onTxTimer(const ros::TimerEvent&) {
  kona_driver_msgs::PrimaryCommandConstPtr primary;
  {
    std::lock_guard<std::mutex> lk(mtx_primary_);
    primary = last_primary_;
  }
  kona_driver_msgs::SecondCommandConstPtr second;
  {
    std::lock_guard<std::mutex> lk(mtx_second_);
    second = last_second_;
  }
  if (!primary && !second) return;
  // Primary 0x157
  if (primary) {
    can_enc::PrimaryCommand157Encoder enc;
    enc.reset();
    enc.UpdateData(
      static_cast<int>(primary->steer_angle_target),
      primary->accel_dec_cmd,
      primary->checksum_157   // ← 토픽에서 온 alive 그대로 사용
    );

    can_msgs::Frame f;
    f.header.stamp = ros::Time::now();
    f.id = can_enc::PrimaryCommand157Encoder::ID;
    f.is_rtr=false; f.is_extended=false; f.is_error=false; f.dlc=8;
    for (int i=0;i<8;i++) f.data[i] = enc.payload[i];
    pub_can_tx_.publish(f);
  }

  if (second) {
    can_enc::Secondcommand156Encoder enc;
    enc.reset();
    enc.UpdateData(
      second->eps_en,
      second->override_ignore,
      second->eps_speed,
      second->acc_en,
      second->aeb_en,
      second->indicator,
      static_cast<std::uint8_t>(second->gear_cmd & 0x0F),
      second->checksum_156
    );

    can_msgs::Frame f;
    f.header.stamp = ros::Time::now();
    f.id = can_enc::Secondcommand156Encoder::ID;
    f.is_rtr=false; f.is_extended=false; f.is_error=false; f.dlc=8;
    for (int i=0;i<8;i++) f.data[i] = enc.payload[i];
    pub_can_tx_.publish(f);
  }
}

} // namespace kona_can
