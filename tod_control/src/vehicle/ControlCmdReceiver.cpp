#include "ControlCmdReceiver.hpp"

#include <ros/serialization.h>
#include <ros/time.h>
#include <arpa/inet.h>   // inet_pton
#include <limits>
#include <iostream>

// ---------------- ctor ----------------
ControlCmdReceiver::ControlCmdReceiver()
{
    // 초기화할 것 거의 없음 (필드는 멤버 초기화 리스트에서 기본값 세팅됨)
}

// --------------- public API ---------------
void ControlCmdReceiver::start(ros::NodeHandle& nh)
{
    ros::NodeHandle pnh("~");
    load_params_(pnh);

    // 퍼블리셔
    control_cmd_pub_ = nh.advertise<tod_msgs::ControlCmd>(control_cmd_topic_, 10);

    // /Vehicle/Manager/status_msg 구독
    status_sub_ = nh.subscribe(status_topic_, 1,
                               &ControlCmdReceiver::on_status_msg, this);

    ROS_INFO("[ControlCmdReceiver] Started. status_topic='%s', control_cmd_topic='%s', mqtt_topic='%s', broker=%s:%d",
             status_topic_.c_str(), control_cmd_topic_.c_str(), mqtt_topic_.c_str(),
             broker_ip_.c_str(), broker_port_);
}

// --------------- 파라미터 로드 ---------------
void ControlCmdReceiver::load_params_(ros::NodeHandle& pnh)
{
    pnh.param<std::string>("status_topic",       status_topic_,       status_topic_);
    pnh.param<std::string>("control_cmd_topic",  control_cmd_topic_,  control_cmd_topic_);
    pnh.param<std::string>("mqtt_topic",         mqtt_topic_,         mqtt_topic_);

    pnh.param<std::string>("broker_ip",         broker_ip_,          broker_ip_);
    pnh.param<int>        ("broker_port",       broker_port_,        broker_port_);
    pnh.param<std::string>("client_id_prefix",  client_id_prefix_,   client_id_prefix_);
    pnh.param<std::string>("mqtt_username",     mqtt_user_,          mqtt_user_);
    pnh.param<std::string>("mqtt_password",     mqtt_pass_,          mqtt_pass_);
}

// --------------- MQTT connect/disconnect ---------------
bool ControlCmdReceiver::check_ip_addr_validity(const std::string& ip_addr) {
    struct in_addr addr{};
    return ::inet_pton(AF_INET, ip_addr.c_str(), &addr) == 1;
}

bool ControlCmdReceiver::create_mqtt_client()
{
    if (!check_ip_addr_validity(broker_ip_)) {
        std::cerr << "[ControlCmdReceiver] Broker IPv4 format invalid: "
                  << broker_ip_ << std::endl;
        return false;
    }

    try {
        // 실제 MqttClientTemplated 생성자 시그니처에 맞게 조정 필요
        // (sender 쪽과 동일한 방식 사용)
        its_client.reset(
            new tod_network::MqttClientTemplated<ControlCmdReceiver>(
                broker_ip_,  // 또는 "tcp://ip:port" 형식 필요 시 mqtt_client_templated 쪽과 맞추기
                client_id_prefix_ + "_receiver"
            )
        );
    } catch (const std::exception& e) {
        std::cerr << "[ControlCmdReceiver] MQTT client create error: " << e.what() << std::endl;
        return false;
    }

    if (its_client && its_client->is_connected()) {
        return true;
    } else {
        std::cerr << "[ControlCmdReceiver] Could not connect to broker: "
                  << broker_ip_ << ":" << broker_port_ << std::endl;
        its_client.reset();
        return false;
    }
}

bool ControlCmdReceiver::disconnect_mqtt_client() {
    if (!its_client) { connected_.store(false); return true; }
    const int rc = its_client->disconnect();
    if (rc == 0) {
        its_client.reset();
        connected_.store(false);
        return true;
    }
    std::cerr << "[ControlCmdReceiver] MQTT disconnect failed (rc=" << rc << ")\n";
    return false;
}

// --------------- MQTT subscribe & callback ---------------
void ControlCmdReceiver::set_mqtt_callback_to_topic(const std::string& mqtt_topic_name) {
    if (!its_client) return;
    its_client->subscribe(mqtt_topic_name, 1,
                          &ControlCmdReceiver::callback_tod_network_client, this);
    ROS_INFO("[ControlCmdReceiver] Subscribed to MQTT topic '%s'", mqtt_topic_name.c_str());
}

void ControlCmdReceiver::callback_tod_network_client(mqtt::const_message_ptr msg) {
    if (!msg) return;

    // MQTT payload -> ROS msg (ControlCmd) 역직렬화
    const auto& payload = msg->get_payload();
    ros::serialization::IStream stream(
        reinterpret_cast<uint8_t*>(const_cast<char*>(payload.data())),
        static_cast<uint32_t>(payload.size()));

    tod_msgs::ControlCmd tmp_cmd;

    try {
        ros::serialization::Serializer<tod_msgs::ControlCmd>::read(stream, tmp_cmd);
    } catch (const std::exception& e) {
        std::cerr << "[ControlCmdReceiver] Deserialization error: " << e.what() << std::endl;
        return;
    }

    // 타임스탬프 업데이트
    tmp_cmd.header.stamp = ros::Time::now();

    // 즉시 publish
    if (control_cmd_pub_) {
        control_cmd_pub_.publish(tmp_cmd);
    }

    // 간단한 카운터 (필요시 유지)
    if (number_received_packages >= std::numeric_limits<int>::max())
        number_received_packages = 0;
    else
        ++number_received_packages;
}

// --------------- Status handler ---------------
void ControlCmdReceiver::on_status_msg(const tod_msgs::Status::ConstPtr& status_msg)
{
    if (!status_msg) return;

    // IDLE → 연결 해제
    if (status_msg->tod_status == tod_msgs::Status::TOD_STATUS_IDLE)
    {
        if (connected_.load())
        {
            ROS_INFO("[ControlCmdReceiver] TOD_STATUS_IDLE: Disconnect MQTT (broker=%s:%d)",
                     broker_ip_.c_str(), broker_port_);
            const bool ok = disconnect_mqtt_client();
            connected_.store(false);
        }
        return;
    }

    // IDLE 이외 상태: 미연결이면 파라미터 기반으로 연결 시도
    if (!connected_.load())
    {
        const bool ok = create_mqtt_client();
        if (ok) {
            connected_.store(true);
            ROS_INFO("[ControlCmdReceiver] MQTT connected to broker: %s:%d",
                     broker_ip_.c_str(), broker_port_);
            set_mqtt_callback_to_topic(mqtt_topic_);
        } else {
            ROS_ERROR("[ControlCmdReceiver] MQTT connect failed to broker: %s:%d",
                      broker_ip_.c_str(), broker_port_);
        }
    }
}