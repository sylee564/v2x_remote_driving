#include "ros/ros.h"
#include "tod_network/tod_client.h"
#include "tod_msgs/PVD_data.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "PVDSender");
    ros::NodeHandle n;

    tod_network::Client<tod_msgs::PVD_data> sender(n, true);
    sender.send_processer("PVD", tod_network::OperatorPorts::RX_VEHICLE_PVD);
    sender.send_run();
    return 0;
}
