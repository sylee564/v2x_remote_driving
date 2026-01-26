#pragma once

#include <ros/ros.h>
#include <ros/serialization.h>
#include <ros/console.h>

#include <tod_msgs/Status.h>
#include <tod_msgs/ControlCmd.h>

#include <atomic>
#include <mutex>
#include <memory>
#include <string>

// MQTT 클라이언트 (서버 주소는 URI: "tcp://<ip>:<port>" 기대)
#include <tod_network/mqtt_client_templated.h>

namespace tod_control {

class ControlCommandSender {
public:
  ControlCommandSender(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~ControlCommandSender();

private:
  // ROS
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_status_;
  ros::Subscriber sub_ctrl_;

  // MQTT
  using MqttClient = tod_network::MqttClientTemplated<ControlCommandSender>;
  std::unique_ptr<MqttClient> its_client;

  // 브로커 설정
  std::string broker_ip_{"127.0.0.1"};
  int         broker_port_{1883};
  std::string client_id_prefix_{"kona_client_sender"};
  std::string mqtt_user_{};   // 현재 구현에서 미사용(보관만)
  std::string mqtt_pass_{};   // 현재 구현에서 미사용(보관만)

  // 상태
  std::atomic<bool> connected_{false};
  std::mutex its_mutex;
  tod_msgs::ControlCmd its_controlCmd_msg;

  // 파라미터/토픽
  std::string status_topic_{"/Operator/Manager/status_msg"};
  std::string cmd_topic_{"/Operator/Control/control_cmd_data"};
  std::string mqtt_topic_{"Vehicle/Control/control_cmd_data"};

private:
  // ROS 콜백
  void on_status_msg(const tod_msgs::Status::ConstPtr& status_msg);
  void on_ctrl_msg(const tod_msgs::ControlCmd::ConstPtr& cmd_msg);

  // MQTT 헬퍼
  bool check_ip_addr_validity(const std::string& ip_addr); // inet_pton 사용
  bool create_mqtt_client(const std::string& client_id, const std::string& ip_addr_broker);
  bool disconnect_mqtt_client();
  void set_mqtt_callback_to_topic(const std::string& topic_name);

  // Publish (ROS 직렬화 바이트 그대로 MQTT 전송)
  void pub_its_control_msg_to_broker(const std::string& topic_name);

  // 기타
  void load_params_();
  void update_time_stamp();
  std::string make_client_id_() const;
};

} // namespace tod_control
