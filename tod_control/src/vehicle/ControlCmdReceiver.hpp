#pragma once

#include <memory>
#include <string>
#include <mutex>
#include <atomic>
#include <cstdint>

#include "ros/ros.h"
#include "std_msgs/UInt16.h"

// Paho C++ 헤더를 *헤더에서* 직접 include 해서 타입 충돌을 방지
#include <mqtt/async_client.h>

#include "tod_network/mqtt_client_templated.h"
#include "tod_msgs/ControlCmd.h"
#include "tod_msgs/Status.h"

/**
 * @brief MQTT에서 수신한 ControlCmd를 즉시 ROS 토픽으로 publish하는 수신기.
 *        (운용자-차량 거리 계산 기능은 제거됨)
 */
class ControlCmdReceiver {
public:
    ControlCmdReceiver();
    ~ControlCmdReceiver() = default;

    ControlCmdReceiver(const ControlCmdReceiver&) = delete;
    ControlCmdReceiver& operator=(const ControlCmdReceiver&) = delete;
    ControlCmdReceiver(ControlCmdReceiver&&) = delete;
    ControlCmdReceiver& operator=(ControlCmdReceiver&&) = delete;

    // 노드 시작: 퍼블리셔/구독자 생성 + 파라미터 로드
    void start(ros::NodeHandle& nh);

    // (호환용) 브로커 제어 API
    bool check_ip_addr_validity(const std::string& ip_addr);
    bool create_mqtt_client();          
    bool disconnect_mqtt_client();

    void set_mqtt_callback_to_topic(const std::string& mqtt_topic_name);

    // MQTT payload 수신 콜백 (MqttClientTemplated에서 바인딩)
    void callback_tod_network_client(mqtt::const_message_ptr msg);

private:
    // /Vehicle/Manager/status_msg 핸들러
    void on_status_msg(const tod_msgs::Status::ConstPtr& status_msg);

    // 내부 메서드
    void load_params_(ros::NodeHandle& pnh);

private:
    // --- ROS ---
    ros::Subscriber status_sub_;           // /Vehicle/Manager/status_msg
    ros::Publisher  control_cmd_pub_;      // ControlCmd publish 토픽

    // --- MQTT 클라이언트 ---
    std::unique_ptr<tod_network::MqttClientTemplated<ControlCmdReceiver>> its_client;

    // --- 상태/동시성 ---
    std::mutex        its_mutex;
    std::atomic<bool> connected_{false};

    // --- 파라미터 (기본값) ---
    std::string status_topic_{"/Vehicle/Manager/status_msg"};
    std::string control_cmd_topic_{"/vehicle/control_cmd_data"};
    std::string mqtt_topic_{"Vehicle/Control/control_cmd_data"};

    std::string broker_ip_{"127.0.0.1"};
    int         broker_port_{1883};
    std::string client_id_prefix_{"kona_client_receiver"};
    std::string mqtt_user_{};
    std::string mqtt_pass_{};

    int number_received_packages{0};
};