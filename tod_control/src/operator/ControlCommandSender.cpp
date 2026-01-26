#include "ControlCommandSender.hpp"
#include <sstream>
#include <arpa/inet.h>  // inet_pton

namespace tod_control {

ControlCommandSender::ControlCommandSender(ros::NodeHandle nh, ros::NodeHandle pnh)
: nh_(std::move(nh)), pnh_(std::move(pnh))
{
  load_params_();

  sub_status_ = nh_.subscribe(status_topic_, 10, &ControlCommandSender::on_status_msg, this);
  sub_ctrl_   = nh_.subscribe(cmd_topic_,    10, &ControlCommandSender::on_ctrl_msg,   this);

  ROS_INFO("[ControlCommandSender] Started. status_topic='%s', cmd_topic='%s', out_mqtt='%s'",
           status_topic_.c_str(), cmd_topic_.c_str(), mqtt_topic_.c_str());
}

ControlCommandSender::~ControlCommandSender(){
  (void)disconnect_mqtt_client();
}

void ControlCommandSender::load_params_(){
  pnh_.param<std::string>("status_topic", status_topic_, status_topic_);
  pnh_.param<std::string>("cmd_topic",    cmd_topic_,    cmd_topic_);
  pnh_.param<std::string>("mqtt_topic",   mqtt_topic_,   mqtt_topic_);

  pnh_.param<int>("broker_port", broker_port_, broker_port_);
  pnh_.param<std::string>("client_id_prefix", client_id_prefix_, client_id_prefix_);

  // 현 구현에선 사용하지 않지만 파라미터로 보관
  pnh_.param<std::string>("mqtt_username", mqtt_user_, mqtt_user_);
  pnh_.param<std::string>("mqtt_password", mqtt_pass_, mqtt_pass_);
}

void ControlCommandSender::on_status_msg(const tod_msgs::Status::ConstPtr& status_msg)
{
  if (!status_msg) return;

  const std::string new_ip = status_msg->operator_broker_ip_address;

  // IDLE → 연결 해제
  if (status_msg->tod_status == tod_msgs::Status::TOD_STATUS_IDLE)
  {
    if (connected_.load())
    {
      ROS_INFO("[ControlCommandSender] TOD_STATUS_IDLE: Disconnect MQTT (broker=%s)", broker_ip_.c_str());
      const bool ok = disconnect_mqtt_client();
      connected_.store(!ok ? true : false);
    }
    return;
  }

  // IP 변경 시 재연결
  if (!new_ip.empty() && new_ip != broker_ip_ && connected_.load())
  {
    ROS_INFO("[ControlCommandSender] Broker IP changed: %s -> %s (reconnect)", broker_ip_.c_str(), new_ip.c_str());
    if (disconnect_mqtt_client())
      connected_.store(false);
  }

  // 미연결이면 연결 시도
  if (!connected_.load() && !new_ip.empty())
  {
    const bool ok = create_mqtt_client(make_client_id_(), new_ip);
    if (ok) {
      broker_ip_ = new_ip;
      connected_.store(true);
      ROS_INFO("[ControlCommandSender] MQTT connected to broker: %s:%d", broker_ip_.c_str(), broker_port_);
      set_mqtt_callback_to_topic(mqtt_topic_);
    } else {
      ROS_ERROR("[ControlCommandSender] MQTT connect failed to broker: %s:%d", new_ip.c_str(), broker_port_);
    }
  }
}

void ControlCommandSender::on_ctrl_msg(const tod_msgs::ControlCmd::ConstPtr& cmd_msg)
{
  if (!cmd_msg) return;

  {
    std::lock_guard<std::mutex> lk(its_mutex);
    its_controlCmd_msg = *cmd_msg; // 최신 명령 보관
  }

  if (connected_.load()) {
    pub_its_control_msg_to_broker(mqtt_topic_);
  } else {
    ROS_WARN_THROTTLE(2.0, "[ControlCommandSender] MQTT not connected yet. Skipping publish.");
  }
}

bool ControlCommandSender::check_ip_addr_validity(const std::string& ip_addr) {
  struct in_addr addr{};
  return ::inet_pton(AF_INET, ip_addr.c_str(), &addr) == 1;
}

bool ControlCommandSender::create_mqtt_client(const std::string& client_id,
                                              const std::string& ip_addr_broker)
{
  if (!check_ip_addr_validity(ip_addr_broker)) {
    ROS_ERROR("[ControlCommandSender] Broker IPv4 format invalid: %s", ip_addr_broker.c_str());
    return false;
  }

  try {
    // 서버 주소는 반드시 URI 형식이어야 합니다: tcp://<ip>:<port>
    const std::string uri = "tcp://" + ip_addr_broker + ":" + std::to_string(broker_port_);
    its_client.reset(new MqttClient(ip_addr_broker, client_id));
  } catch (const std::exception& e) {
    ROS_ERROR("[ControlCommandSender] MQTT client create error: %s", e.what());
    return false;
  }

  if (its_client && its_client->is_connected()) {
    return true;
  } else {
    ROS_ERROR("[ControlCommandSender] Could not connect to broker: %s:%d",
              ip_addr_broker.c_str(), broker_port_);
    its_client.reset();
    return false;
  }
}

bool ControlCommandSender::disconnect_mqtt_client()
{
  try {
    if (its_client) {
      bool ok = its_client->disconnect();
      its_client.reset();
      return ok;
    }
  } catch (const std::exception& e) {
    ROS_ERROR("[ControlCommandSender] disconnect_mqtt_client exception: %s", e.what());
  }
  return true;
}

void ControlCommandSender::set_mqtt_callback_to_topic(const std::string& topic_name)
{
  ROS_INFO("[ControlCommandSender] Ready to publish to topic '%s'", topic_name.c_str());
}

void ControlCommandSender::pub_its_control_msg_to_broker(const std::string& topic_name)
{
  if (!its_client) return;

  update_time_stamp();

  // its_mutex.lock();
  ros::SerializedMessage rosSer =
      ros::serialization::serializeMessage<tod_msgs::ControlCmd>(its_controlCmd_msg);
  // its_mutex.unlock();

  // mqtt_client_templated.h 의 publish가 int(0=성공)라고 가정
  const int rc = its_client->publish(topic_name, /*qos=*/1,
                                     reinterpret_cast<char*>(rosSer.message_start),
                                     rosSer.num_bytes);
  if (rc != 0) {
    ROS_WARN_THROTTLE(2.0, "[ControlCommandSender] MQTT publish failed (topic=%s, bytes=%zu, rc=%d)",
                      topic_name.c_str(), (size_t)rosSer.num_bytes, rc);
  } else {
    ROS_DEBUG("[ControlCommandSender] Published ControlCmd (%zu bytes) → %s",
              (size_t)rosSer.num_bytes, topic_name.c_str());
  }
}

void ControlCommandSender::update_time_stamp(){
  std::lock_guard<std::mutex> lk(its_mutex);
  its_controlCmd_msg.header.stamp = ros::Time::now();
}

std::string ControlCommandSender::make_client_id_() const {
  std::ostringstream oss;
  oss << client_id_prefix_ << "_" << ros::this_node::getName();
  return oss.str();
}

} // namespace tod_control
