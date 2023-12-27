#pragma once
#include <ros/ros.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <thread>
#include "connection_configs.h"

#define MAXLINE 100*1024

namespace tod_network {

class UdpClient {
public:
    UdpClient(const std::string& destIPAddress, const int& destPort);
    ~UdpClient() { close(sockfd); }

    template <class RecvMsg> 
    int receive_ros_msg( RecvMsg& msg) {
        int integer{0};
        send((char *)&integer, sizeof(integer));
        char buffer[MAXLINE];
        // socklen_t addrlen=sizeof(servaddr);
        // int recvBytes;
        int recvBytes = recvfrom(sockfd, (char *)buffer, MAXLINE, MSG_WAITALL, (struct sockaddr *) &servaddr, &addrlen);
        try {
            // Reconstruct ros message object
            ros::serialization::IStream stream((uint8_t*) buffer, recvBytes);
            ros::serialization::Serializer<RecvMsg>::read(stream, msg);
        } catch (const ros::serialization::StreamOverrunException& e ) {
            ROS_DEBUG("Error during message reconstruction");
        }
        return recvBytes;
    }

    template <class SendMsg>
    int send_ros_msg(const SendMsg& msg) {

        ros::SerializedMessage serMsg = ros::serialization::serializeMessage(msg);
        // printf("####sending to Server IP : %s, Port: %i\n", inet_ntoa(servaddr.sin_addr), ntohs(servaddr.sin_port));


        return sendto(sockfd, serMsg.message_start, serMsg.num_bytes, 0,
                      (struct sockaddr *)&servaddr, sizeof(servaddr));
    }

    template <class Cfg>
    int send_ros_config(const Cfg& cfg) {
        dynamic_reconfigure::Config msg;
        cfg.__toMessage__(msg);
        ros::SerializedMessage serMsg = ros::serialization::serializeMessage(msg);
        return sendto(sockfd, serMsg.message_start, serMsg.num_bytes, 0,
                      (struct sockaddr *)&servaddr, sizeof(servaddr));
    }

    int receive_data(char *msg) { return recvfrom(sockfd, msg, MAXLINE, MSG_WAITALL, (struct sockaddr *) &servaddr, &addrlen); }

    int send(const char *msg, size_t size) {
        printf("##Sending to Server IP : %s, Port: %i\n", inet_ntoa(servaddr.sin_addr), ntohs(servaddr.sin_port));

        return sendto(sockfd, msg, size, 0, (struct sockaddr *)&servaddr, sizeof(servaddr));
    }

    void change_destination(const std::string& destIPAddress, const int& destPort = -1);
    void print_connection_specs();

private:
    struct sockaddr_in servaddr;
    socklen_t addrlen=sizeof(servaddr);
    int sockfd;
    
};
}; // namespace tod_network