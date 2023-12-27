#pragma once
#include <ros/ros.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <arpa/inet.h>
#include <thread>
#include "udp_client.h"
#include "connection_configs.h"

#define MAXLINE 100*1024

namespace tod_network {

class UdpServer {
public:
    explicit UdpServer(const int destPort);
    ~UdpServer() { close(sockfd); }

    template <class RecvMsg>
    int receive_ros_msg(RecvMsg& msg) {
        char buffer[MAXLINE];
        // socklen_t addrlen=sizeof(cliaddr);
        int recvBytes = recvfrom(sockfd, (char *)buffer, MAXLINE, MSG_WAITALL, ( struct sockaddr *) &cliaddr, &addrlen);
        // printf("recevied : %d\n", n);

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
        char buffer[MAXLINE];
        // socklen_t addrlen=sizeof(cliaddr);

        // printf("Client IP : %s, Port: %i\n", inet_ntoa(cliaddr.sin_addr), ntohs(cliaddr.sin_port));

        ros::SerializedMessage serMsg = ros::serialization::serializeMessage(msg);
        return sendto(sockfd, serMsg.message_start, serMsg.num_bytes, 0,
                      (struct sockaddr *)&cliaddr, sizeof(cliaddr));
    }

    int receive_data() { 
        char buffer;
        // socklen_t addrlen=sizeof(cliaddr);
        return recvfrom(sockfd, &buffer, sizeof(buffer), MSG_WAITALL, ( struct sockaddr *) &cliaddr, &addrlen); 
    }
    int wait_for_udp_receiver_to_close();
    void change_destination(const std::string& IPAddress, const int& destPort = -1);

private:
    struct sockaddr_in servaddr, cliaddr;
    int sockfd;
    socklen_t addrlen=sizeof(cliaddr);
    int n;
    std::thread anti_block_thread;
    UdpClient anti_block_sender;

    void send_data_for_anti_block();
};
}; //namespace tod_network