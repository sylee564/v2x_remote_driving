#pragma once
#include <ros/ros.h>
#include <tod_msgs/Status.h>
#include "tod_network/udp_server.h"
#include <tod_msgs/PacketInfo.h>
#include <memory>
#include <algorithm>
#include <thread>

namespace tod_network {

template <typename T>
class Server {
private:
    struct RosMsgSender {
        ros::Subscriber sendMsgSubs;
        std::unique_ptr<tod_network::UdpServer> udpServer{nullptr};
        bool printedInfo{false};
        bool recveiveFlag{false};
    };

    struct RosMsgReceiver{
        ros::Publisher recvMsgPubs, recvMsgPacketInfoPubs;
        std::unique_ptr<tod_network::UdpServer> udpServer{nullptr};
        bool printedInfo{false};
        std::unique_ptr<std::thread> thread{nullptr};
        T msg;
        tod_msgs::PacketInfo packetInfoMsg;
    };

public:
    Server(ros::NodeHandle &n, int port, bool restamp = false) : _n(n) {
        receive_processer("/received_topic", port, restamp);
    }
    Server(ros::NodeHandle &n, bool senderInVehicle)
        : _n(n), _senderInVehicle(senderInVehicle) {
        std::string statusTopic =
            (_senderInVehicle) ? "/Vehicle/Manager/status_msg" : "/Operator/Manager/status_msg";
        _statusSubs = _n.subscribe(statusTopic, 1, &Server::statusMessageReceived, this);
    }
    explicit Server(ros::NodeHandle &n) : _n(n) {}

    void receive_processer(const std::string &topic, const int port, const bool restamp = false) {
        auto receiver = _rosMsgReceivers.emplace_back(std::make_shared<RosMsgReceiver>());
        receiver->udpServer = std::make_unique<tod_network::UdpServer>(port);
        receiver->recvMsgPubs = _n.advertise<T>(topic, 1);
        receiver->recvMsgPacketInfoPubs = _n.advertise<tod_msgs::PacketInfo>(topic + "_packet_info", 1);
    }

    void send_processer(const std::string &topic, const int port) {
        _serverPort = port;
        auto sender = _rosMsgSenders.emplace_back(std::make_shared<RosMsgSender>());
        sender->udpServer = std::make_unique<tod_network::UdpServer>(port);
        sender->sendMsgSubs = _n.subscribe<T>(
            topic, 10, boost::bind(&Server::sendMessageReceived, this, _1, sender));
    }

    void receive() {
        for (auto receiver : _rosMsgReceivers)
            receiver->thread = std::make_unique<std::thread>(&Server::receive_msgs, this, receiver);
    }

    void receive_run() {
        receive();
        ros::spin();
    }

    void send_run() const { ros::spin(); }

    void send_in_control_mode(const uint8_t mode) {
        _sendingControlModes.push_back(mode);
    }

private:
    ros::NodeHandle &_n;
    ros::Subscriber _statusSubs;
    std::string _receiverIp{"127.0.0.1"};
    std::string _senderIp{"127.0.0.1"};
    int _serverPort;
    std::vector<std::shared_ptr<RosMsgSender>> _rosMsgSenders;
    std::vector<std::shared_ptr<RosMsgReceiver>> _rosMsgReceivers;
    uint8_t _connectionStat{tod_msgs::Status::TOD_STATUS_IDLE};
    uint8_t _controlMode;
    bool _senderInVehicle;
    std::vector<uint8_t> _sendingControlModes;

    bool connected() {
        return _connectionStat != tod_msgs::Status::TOD_STATUS_IDLE;
    }

    void statusMessageReceived(const tod_msgs::Status &msg) {
        _connectionStat = msg.tod_status;
        _controlMode = _senderInVehicle ? msg.vehicle_control_mode : msg.operator_control_mode;
        if (connected() && in_sending_control_mode()) {
            if (_senderInVehicle && _receiverIp != msg.operator_PC_ip_address) {
                _receiverIp = msg.operator_PC_ip_address;
                ROS_INFO("%s: sending to operator ip address %s",
                         ros::this_node::getName().c_str(), _receiverIp.c_str());
                // for (auto rosMsgSender : _rosMsgSenders)
                //     rosMsgSender->udpServer->change_destination(_receiverIp);
            } else if (!_senderInVehicle && _receiverIp != msg.vehicle_device_ip_address) {
                _receiverIp = msg.vehicle_device_ip_address;
                _senderIp = msg.operator_PC_ip_address;
                ROS_INFO("%s: sending to vehicle ip address %s",
                         ros::this_node::getName().c_str(), _receiverIp.c_str());
                // for (auto rosMsgSender : _rosMsgSenders)
                //     rosMsgSender->udpServer->change_destination(_senderIp, _serverPort);
            }
        }
    }

    void sendMessageReceived(const ros::MessageEvent<T const>& event, std::shared_ptr<RosMsgSender> rosMsgSender) {
        if (connected()) {
            if(!rosMsgSender->recveiveFlag){
                printf("client msg receiving ...\n");
                int recvBytes = rosMsgSender->udpServer->receive_data();
                if(recvBytes > 0){
                    
                    rosMsgSender->recveiveFlag = true;
                    printf("recieved bytes :%d\n", recvBytes);
                }
            }
            else{
                int nofBytesSent = rosMsgSender->udpServer->send_ros_msg(*event.getMessage());
                if (!rosMsgSender->printedInfo) {
                    ROS_INFO("%s: sending topic %s - msg size %d", ros::this_node::getName().c_str(),
                            rosMsgSender->sendMsgSubs.getTopic().c_str(), nofBytesSent);
                    rosMsgSender->printedInfo = true;
                }
            }
        }
        else{
            rosMsgSender->recveiveFlag = false;
            rosMsgSender->printedInfo = false;
        }
    }

    void receive_msgs(std::shared_ptr<RosMsgReceiver> receiver) {
        while (ros::ok()) {
            int sizeByte = receiver->udpServer->receive_ros_msg(receiver->msg);
            // publish packet info msg  
            receiver->packetInfoMsg.header.stamp = ros::Time::now();
            receiver->packetInfoMsg.seqNum = receiver->msg.header.seq;
            receiver->packetInfoMsg.sizeBit = sizeByte * 8;
            receiver->packetInfoMsg.latencyUsec = int64_t(ros::Time::now().toNSec()/1000
                                                         - receiver->msg.header.stamp.toNSec()/1000);
            receiver->recvMsgPacketInfoPubs.publish(receiver->packetInfoMsg);

            // publish restamped data msg
            receiver->msg.header.stamp = ros::Time::now();
            receiver->recvMsgPubs.publish(receiver->msg);
            if (!receiver->printedInfo) {
                receiver->printedInfo = true;
                ROS_INFO("%s: receiving topic %s", ros::this_node::getName().c_str(),
                         receiver->recvMsgPubs.getTopic().c_str());
            }
        }
    }

    bool in_sending_control_mode(){
        if (_sendingControlModes.size() == 0)
            return true;

        return std::any_of(_sendingControlModes.begin(), _sendingControlModes.end(),
            [this](uint8_t sendingMode){ return _controlMode == sendingMode; });
    }
};
}; // namespace tod_network