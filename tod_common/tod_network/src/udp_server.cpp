#include "tod_network/udp_server.h"

namespace tod_network {

UdpServer::UdpServer(const int destPort)
    : anti_block_thread{}, anti_block_sender("127.0.0.1", destPort) {
    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));
    // change_destination(IPAddress, destPort);
    // creating socket file descriptor
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(destPort);
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    // servaddr.sin_addr.s_addr = inet_addr(IPAddress.c_str());
    if ( (sockfd = socket(servaddr.sin_family, SOCK_DGRAM, 0)) < 0 ) {
        perror("socket creation failed");
        printf("port %i", destPort);
        exit(EXIT_FAILURE);
    }
    // bind the socket with the server address
    if ( bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0 ) {
        perror("bind failed");
        printf("port %i", destPort);
        exit(EXIT_FAILURE);
    }
    anti_block_thread = std::thread(&UdpServer::send_data_for_anti_block, this);
    printf("###Operator open#### Server IP : %s, Port: %i\n", inet_ntoa(servaddr.sin_addr), ntohs(servaddr.sin_port));

}

void UdpServer::change_destination(const std::string &IPAddress, const int &destPort) {
    // fill servaddr with values
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(destPort);
    servaddr.sin_addr.s_addr = inet_addr(IPAddress.c_str());
    printf("Server IP : %s, Port: %i\n", inet_ntoa(servaddr.sin_addr), ntohs(servaddr.sin_port));

}

void UdpServer::send_data_for_anti_block() {
    if (!ros::ok()) {
        printf("\033[31m ERROR @ UdpServer::send_data_for_anti_block() port %i: "
               " UDP-RECEIVER NEEDS TO BE initialized AFTER ros::init() \033[0m \n", servaddr.sin_port);
    }

    while (ros::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    // free block of receiver
    int integer { 0 };
    for (int it = 0; it != 20; ++it)
        anti_block_sender.send((char *) &integer, sizeof(integer));
}

int UdpServer::wait_for_udp_receiver_to_close() {
    anti_block_thread.join();
    return 0;
}
}; // namespace tod_network