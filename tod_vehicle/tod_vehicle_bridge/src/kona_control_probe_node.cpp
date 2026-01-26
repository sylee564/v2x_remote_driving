#include "tod_vehicle_bridge/kona_control_probe_node.hpp"

namespace tod_kona_bridge {

static inline double rad2deg(double rad) { return rad * 180.0 / M_PI; }

KonaControlProbeNode::KonaControlProbeNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
: nh_(nh), pnh_(pnh)
{
  // -------- parameters --------
  loop_rate_hz_  = pnh_.param("loop_rate_hz", 30.0);
  speed_is_kmh_  = pnh_.param("speed_is_kmh", true);

  topic_cmd_     = pnh_.param<std::string>("control_cmd_topic", "/Vehicle/Control/control_cmd_data");

  topic_primary_ = pnh_.param<std::string>("primary_command_topic", "primary_command");
  topic_second_  = pnh_.param<std::string>("second_command_topic",  "second_command");

  topic_steer_   = pnh_.param<std::string>("steer_report_topic", "/Vehicle/kona/interface/steer_report");
  topic_acc_     = pnh_.param<std::string>("acc_report_topic",   "/Vehicle/kona/interface/acc_report");
  topic_brake_   = pnh_.param<std::string>("brake_report_topic", "/Vehicle/kona/interface/brake_report");
  topic_wheel_   = pnh_.param<std::string>("wheel_report_topic", "/Vehicle/kona/interface/wheel_report");
  topic_gear_    = pnh_.param<std::string>("gear_report_topic",  "/Vehicle/kona/interface/gear_report");

  topic_fix_     = pnh_.param<std::string>("fix_topic",     "/fix");
  topic_heading_ = pnh_.param<std::string>("heading_topic", "/heading");

  topic_pvd_     = pnh_.param<std::string>("pvd_topic", "probe_vehicle_data");

  vehicle_name_  = pnh_.param<std::string>("vehicleName", "");
  vehicle_id_    = pnh_.param<std::string>("vehicleNum", "");
  vehicle_type_  = static_cast<std::uint16_t>(pnh_.param("vehicleType", 0));

  cache_.vehicle_name = vehicle_name_;
  cache_.vehicle_id   = vehicle_id_;
  cache_.vehicle_type = vehicle_type_;

  // Speed governor params
  gov_param_.enabled   = pnh_.param("gov_enabled", true);
  gov_param_.v_max_mps = pnh_.param("gov_vmax_mps", 16.7);
  gov_param_.hyst_mps  = pnh_.param("gov_hyst_mps", 0.7);
  gov_param_.Kp        = pnh_.param("gov_Kp", 1.5);
  gov_param_.a_min     = pnh_.param("gov_a_min", -3.0);
  gov_param_.a_max     = pnh_.param("gov_a_max", +1.0);
  governor_.setParams(gov_param_);

  eps_speed_    = pnh_.param("eps_speed", +1.0);


  // -------- publishers --------
  pub_primary_ = nh_.advertise<kona_driver_msgs::PrimaryCommand>(topic_primary_, 1);
  pub_second_  = nh_.advertise<kona_driver_msgs::SecondCommand>(topic_second_, 1);
  pub_pvd_     = nh_.advertise<tod_msgs::ProbeVehicleData>(topic_pvd_, 1);

  // -------- subscribers --------
  sub_cmd_   = nh_.subscribe(topic_cmd_, 1, &KonaControlProbeNode::onControlCmd, this);

  sub_steer_ = nh_.subscribe(topic_steer_, 1, &KonaControlProbeNode::onSteer, this);
  sub_acc_   = nh_.subscribe(topic_acc_,   1, &KonaControlProbeNode::onAcc, this);
  sub_brake_ = nh_.subscribe(topic_brake_, 1, &KonaControlProbeNode::onBrake, this);
  sub_wheel_ = nh_.subscribe(topic_wheel_, 1, &KonaControlProbeNode::onWheel, this);
  sub_gear_  = nh_.subscribe(topic_gear_,  1, &KonaControlProbeNode::onGear, this);

  sub_fix_     = nh_.subscribe(topic_fix_,     1, &KonaControlProbeNode::onFix, this);
  sub_heading_ = nh_.subscribe(topic_heading_, 1, &KonaControlProbeNode::onHeading, this);

  // -------- timer --------
  const double period = 1.0 / std::max(1.0, loop_rate_hz_);
  timer_ = nh_.createTimer(ros::Duration(period), &KonaControlProbeNode::onTimer, this);

  ROS_INFO_STREAM("kona_control_probe_node up"
    << " | cmd=" << topic_cmd_
    << " | primary=" << topic_primary_
    << " | second=" << topic_second_
    << " | pvd=" << topic_pvd_
    << " | rate=" << loop_rate_hz_ << "Hz");
}

// ---------- Callbacks ----------
void KonaControlProbeNode::onControlCmd(const tod_msgs::ControlCmd::ConstPtr& m) {
  if (!m) return;
  std::lock_guard<std::mutex> lk(mtx_);
  control_.target_steer_angle = static_cast<int>(m->control.steering_angle);
  control_.steering_angle_velocity = static_cast<int>(m->control.steering_angle_velocity);
  control_.target_indicator   = m->indicator.data;
  control_.target_gear        = m->shift.data;
  control_.remote_flag        = m->remote_flag;
  control_.aeb_flag           = m->aeb_flag;

  // [-3, +1], floor to 0.01
  control_.target_acceleration = clampAccelFloor2(static_cast<float>(m->acceleration));
}

void KonaControlProbeNode::onSteer(const kona_driver_msgs::SteerReport::ConstPtr& m) {
  if (!m) return;
  std::lock_guard<std::mutex> lk(mtx_);
  cache_.steer = *m;
  if (!printed_first_data_) {
    printed_first_data_ = true;
    ROS_INFO("%s: Received first kona data from can interface!", ros::this_node::getName().c_str());
  }
}

void KonaControlProbeNode::onAcc(const kona_driver_msgs::AccReport::ConstPtr& m)   { 
  if (!m) return; 
  std::lock_guard<std::mutex> lk(mtx_); 
  cache_.acc = *m; 
}

void KonaControlProbeNode::onBrake(const kona_driver_msgs::BrakeReport::ConstPtr& m){ 
  if (!m) return; 
  std::lock_guard<std::mutex> lk(mtx_); 
  cache_.brake = *m; 
}

void KonaControlProbeNode::onWheel(const kona_driver_msgs::WheelSpeedReport::ConstPtr& m){
  if (!m) return; 
  std::lock_guard<std::mutex> lk(mtx_); 
  cache_.wheel = *m; 
}

void KonaControlProbeNode::onGear(const kona_driver_msgs::GearReport::ConstPtr& m) { 
  if (!m) return; 
  std::lock_guard<std::mutex> lk(mtx_); 
  cache_.gear = *m; 
}

void KonaControlProbeNode::onFix(const sensor_msgs::NavSatFix::ConstPtr& m) {
  if (!m) return;
  std::lock_guard<std::mutex> lk(mtx_);
  cache_.lat = m->latitude;
  cache_.lon = m->longitude;
  cache_.alt = static_cast<float>(m->altitude);

  // GNSS status 매핑 (간단 규칙):
  // NavSatStatus::STATUS_NO_FIX(-1) → 0
  // STATUS_FIX(0) → 1
  // STATUS_SBAS_FIX(1), STATUS_GBAS_FIX(2) → 2
  int8_t st = m->status.status;
  if (st < 0) cache_.gnss_status = 0;
  else if (st == 0) cache_.gnss_status = 1;
  else cache_.gnss_status = 2;
}

void KonaControlProbeNode::onTodStatus(const tod_msgs::Status::ConstPtr& m) {
  if (!m) return;
  tod_status_ = m->tod_status;
  // mode_ = m->mode;
}

void KonaControlProbeNode::onHeading(const geometry_msgs::QuaternionStamped::ConstPtr& m) {
  if (!m) return;
  std::lock_guard<std::mutex> lk(mtx_);
  tf::Quaternion q(m->quaternion.x, m->quaternion.y, m->quaternion.z, m->quaternion.w);
  tf::Matrix3x3 R(q);
  double r,p,y; 
  R.getRPY(r,p,y);
  float hdg = static_cast<float>(rad2deg(y));
  while (hdg < 0.0f)   hdg += 360.0f;
  while (hdg >= 360.0f) hdg -= 360.0f;
  cache_.heading_deg = hdg;
}

// ---------- Periodic ----------
void KonaControlProbeNode::onTimer(const ros::TimerEvent&) {
  DirectControlOutput ctrl;
  ProbeVehicleDataCache c;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    ctrl = control_;
    c    = cache_;
  }

  // CAN 진입 직전: 2차 속도 상한 거버너 적용
  double v_meas = static_cast<double>(c.acc.vehicle_speed); // 원래 메시지가 uint8 raw
  if (speed_is_kmh_) v_meas = VH::Model::kph2mps(v_meas);

  const double a_in    = static_cast<double>(ctrl.target_acceleration);
  const double a_safe  = governor_.apply(a_in, v_meas);
  ctrl.target_acceleration = static_cast<float>(a_safe);

  // PrimaryCommand publish
  kona_driver_msgs::PrimaryCommand primary;
  buildPrimaryCommand(ctrl, primary);
  pub_primary_.publish(primary);

  // SecondCommand publish (eps_speed 포함)
  kona_driver_msgs::SecondCommand second;
  buildSecondCommand(ctrl, c, second);
  pub_second_.publish(second);

  // ProbeVehicleData publish (새 규격에 맞게 작성)
  tod_msgs::ProbeVehicleData pvd;
  buildProbeVehicle(c, pvd);
  pub_pvd_.publish(pvd);
}

// ---------- Builders ----------
void KonaControlProbeNode::buildPrimaryCommand(const DirectControlOutput& in,
                                               kona_driver_msgs::PrimaryCommand& out)
{
  // if(tod_status_ != tod_msgs::Status::TOD_STATUS_TELEOPERATION /*|| mode_ != tod_msgs::Status::MODE_REMOTE */) return;
  
  static int seq{0};
  out.header.seq   = ++seq;
  out.header.stamp = ros::Time::now();
  out.steer_angle_target = in.target_steer_angle;
  out.accel_dec_cmd      = in.target_acceleration;
  checksum_157_ = static_cast<std::uint8_t>(checksum_157_ + 1u);
  out.checksum_157 = checksum_157_;
}

void KonaControlProbeNode::buildSecondCommand(const DirectControlOutput& in,
                                              const ProbeVehicleDataCache& c,
                                              kona_driver_msgs::SecondCommand& out)
{
  // if(tod_status_ != tod_msgs::Status::TOD_STATUS_TELEOPERATION) return;

  static int seq{0};
  out.header.seq   = ++seq;
  out.header.stamp = ros::Time::now();

  // enable flags
  if (in.remote_flag) { out.acc_en = 1; out.eps_en = 1; }
  else                { out.acc_en = 0; out.eps_en = 0; }

  out.aeb_en    = in.aeb_flag ? 1 : 0;
  out.override_ignore = 0;
  out.indicator = in.target_indicator;
  out.gear_cmd  = in.target_gear;


  out.eps_speed = 150;
  // out.eps_speed = in.steering_angle_velocity;

  checksum_156_ = static_cast<std::uint8_t>(checksum_156_ + 1u);
  out.checksum_156 = checksum_156_;
}

void KonaControlProbeNode::buildProbeVehicle(const ProbeVehicleDataCache& t, tod_msgs::ProbeVehicleData& out)
{
  static int seq{0};
  out.header.seq      = ++seq;
  out.header.stamp    = ros::Time::now();
  out.header.frame_id = "probe_vehicle_data";

  // identity
  out.vehicle_name = t.vehicle_name;
  out.vehicle_id   = t.vehicle_id;
  out.vehicle_type = t.vehicle_type;

  // pose / status
  out.longitude = t.lon;
  out.latitude  = t.lat;
  out.altitude  = t.alt;
  out.heading   = t.heading_deg;

  // vehicle dynamics (새 메시지 필드 매핑)
  out.steering_wheel = t.steer.steering_angle; 
  out.acceleration   = static_cast<float>(t.acc.long_accel);                         // m/s^2 (측정치)
  out.velocity       = static_cast<uint8_t>(t.acc.vehicle_speed);                    // raw/kmh 가정
  out.gear_status    = static_cast<uint8_t>(t.gear.gear_current);

  out.vehicle_mile   = t.vehicle_mile;

  // mode
  out.vehicle_mode_status = (t.acc.acc_en_status==1 && t.steer.eps_en_status==1) ? 2 : 0;

  // subsystems status
  // out.steer_status = static_cast<uint8_t>(t.steer.eps_control_status);  // EPS 상태 코드
  // out.acc_status = static_cast<uint8_t>(t.acc.acc_control_status);    // ACC 상태 코드

  // sensors
  out.gnss_status = t.gnss_status;
  out.camera_status.clear(); // 카메라 상태가 따로 들어오면 여기에 push_back 하세요.
}

// ---------- Helpers ----------
inline float KonaControlProbeNode::clampAccelFloor2(float a) {
  if (a < -3.0f) a = -3.0f;
  else if (a >  1.0f) a = 1.0f;
  // floor to 0.01
  return std::floor(a * 100.0f) / 100.0f;
}

} // namespace tod_kona_bridge
