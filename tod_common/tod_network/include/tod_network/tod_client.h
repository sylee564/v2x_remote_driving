#pragma once
#include <ros/ros.h>
#include <tod_msgs/Status.h>
#include <tod_network/udp_client.h>
#include <tod_msgs/PacketInfo.h>
#include <memory>
#include <algorithm>

namespace tod_network {

template <typename T>
class Client {
private:
    struct RosMsgSender {
        ros::Subscriber sendMsgSubs;
        std::unique_ptr<tod_network::UdpClient> udpClient{nullptr};
        bool printedInfo{false};
    };

    struct RosMsgReceiver{
        ros::Publisher recvMsgPubs, recvMsgPacketInfoPubs;
        std::unique_ptr<tod_network::UdpClient> udpClient{nullptr};
        bool printedInfo{false};
        std::unique_ptr<std::thread> thread{nullptr};
        T msg;
        tod_msgs::PacketInfo packetInfoMsg;
        bool sendFlag{false};
    };

public:
    Client(ros::NodeHandle &n, int port, bool senderInVehicle)
        : _n(n), _senderInVehicle(senderInVehicle) {
        std::string statusTopic =
            (_senderInVehicle) ? "/Vehicle/Manager/status_msg" : "/Operator/Manager/status_msg";
        _statusSubs = _n.subscribe(statusTopic, 1, &Client::statusMessageReceived, this);
        send_processer("/topic_to_send", port);
    }
    Client(ros::NodeHandle &n, bool senderInVehicle)
        : _n(n), _senderInVehicle(senderInVehicle) {
        std::string statusTopic =
            (_senderInVehicle) ? "/Vehicle/Manager/status_msg" : "/Operator/Manager/status_msg";
        _statusSubs = _n.subscribe(statusTopic, 1, &Client::statusMessageReceived, this);
        
    }

    void receive_processer(const std::string &topic, const int port) {
        _receiverPort = port;
        auto receiver = _rosMsgReceivers.emplace_back(std::make_shared<RosMsgReceiver>());
        receiver->udpClient = std::make_unique<tod_network::UdpClient>(_serverIp, port);
        receiver->recvMsgPubs = _n.advertise<T>(topic, 1);
        receiver->recvMsgPacketInfoPubs = _n.advertise<tod_msgs::PacketInfo>(topic + "_packet_info", 1);
    }

    void receive() {
        for (auto receiver : _rosMsgReceivers)
            receiver->thread = std::make_unique<std::thread>(&Client::receive_msgs, this, receiver);
    }

    void receive_run() {
        receive();
        ros::spin();
    }

    void send_processer(const std::string &topic, const int port) {
        auto sender = _rosMsgSenders.emplace_back(std::make_shared<RosMsgSender>());
        sender->udpClient = std::make_unique<tod_network::UdpClient>(_serverIp, port);
        sender->sendMsgSubs = _n.subscribe<T>(
            topic, 10, boost::bind(&Client::sendMessageReceived, this, _1, sender));
    }
    

    void send_run() const { ros::spin(); }

    void send_in_control_mode(const uint8_t mode) {
        _sendingControlModes.push_back(mode);
    }

private:
    ros::NodeHandle &_n;
    std::string _serverIp{"127.0.0.1"};
    int _receiverPort;
    std::vector<std::shared_ptr<RosMsgSender>> _rosMsgSenders;
    std::vector<std::shared_ptr<RosMsgReceiver>> _rosMsgReceivers;
    uint8_t _connectionStat{tod_msgs::Status::TOD_STATUS_IDLE};
    uint8_t _controlMode;
    ros::Subscriber _statusSubs;
    bool _senderInVehicle;
    std::vector<uint8_t> _sendingControlModes;

    bool connected() {
        return _connectionStat != tod_msgs::Status::TOD_STATUS_IDLE;
    }

    void statusMessageReceived(const tod_msgs::Status &msg) {
        _connectionStat = msg.tod_status;
        _controlMode = _senderInVehicle ? msg.vehicle_control_mode : msg.operator_control_mode;
        if (connected() && in_sending_control_mode()) {
            if (_senderInVehicle && _serverIp != msg.vehicle_device_ip_address) {
                _serverIp = msg.vehicle_device_ip_address;
                ROS_INFO("%s: sending to operator ip address %s",
                         ros::this_node::getName().c_str(), _serverIp.c_str());
                for (auto rosMsgSender : _rosMsgSenders)
                    rosMsgSender->udpClient->change_destination(_serverIp);
                for (auto rosMsgReceiver : _rosMsgReceivers)
                    rosMsgReceiver->udpClient->change_destination(_serverIp, _receiverPort);
            } else if (!_senderInVehicle && _serverIp != msg.vehicle_device_ip_address) {
                _serverIp = msg.vehicle_device_ip_address;
                ROS_INFO("%s: received from operator ip address %s",
                         ros::this_node::getName().c_str(), _serverIp.c_str());
                for (auto rosMsgSender : _rosMsgSenders)
                    rosMsgSender->udpClient->change_destination(_serverIp);
                // for (auto rosMsgReceiver : _rosMsgReceivers)
                //     rosMsgReceiver->udpClient->change_destination(_serverIp);
            }
        }
    }

    void sendMessageReceived(const ros::MessageEvent<T const>& event, std::shared_ptr<RosMsgSender> rosMsgSender) {
        if (connected() && in_sending_control_mode()) {
            int nofBytesSent = rosMsgSender->udpClient->send_ros_msg(*event.getMessage());
            // printf("send byte : %d\n", nofBytesSent);
            if (!rosMsgSender->printedInfo) {
                ROS_INFO("%s: sending topic %s - msg size %d", ros::this_node::getName().c_str(),
                         rosMsgSender->sendMsgSubs.getTopic().c_str(), nofBytesSent);
                rosMsgSender->printedInfo = true;
            }
        }
    }

    void receive_msgs(std::shared_ptr<RosMsgReceiver> receiver) {
        uint8_t msg = 0;
        while (ros::ok()) {
            
            if(connected()){
                msg = 1;
                // printf("connect sts: %d\n",connected());
                if(!receiver->sendFlag){
                    std::this_thread::sleep_for(std::chrono::milliseconds(250));

                    int sendByte = receiver->udpClient->send((char *) &msg, sizeof(msg));
                    
                }

                int sizeByte = receiver->udpClient->receive_ros_msg(receiver->msg);
                // printf("received bytes : %d\n", sizeByte);
                if(sizeByte>0){
                    
                    receiver->sendFlag = true;
                }
                // int sizeByte; 
                // publish packet info msg
                receiver->packetInfoMsg.header.stamp = ros::Time::now();
                receiver->packetInfoMsg.seqNum = receiver->msg.header.seq;
                receiver->packetInfoMsg.sizeBit = sizeByte * 8;
                receiver->packetInfoMsg.latencyUsec = int64_t(ros::Time::now().toNSec()/1000
                                                            - receiver->msg.header.stamp.toNSec()/1000);
                receiver->recvMsgPacketInfoPubs.publish(receiver->packetInfoMsg);

                // // publish restamped data msg
                receiver->msg.header.stamp = ros::Time::now();
                receiver->recvMsgPubs.publish(receiver->msg);
                if (!receiver->printedInfo) {
                    receiver->printedInfo = true;
                    ROS_INFO("%s: receiving topic %s", ros::this_node::getName().c_str(),
                            receiver->recvMsgPubs.getTopic().c_str());
                }

            }
            else{
                // printf("connect sts: %d\n", connected());
                msg = 0;
                receiver->sendFlag = false;
                receiver->printedInfo = false;
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