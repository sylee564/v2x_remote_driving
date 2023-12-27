#include <ros/ros.h>
#include "tod_network/tod_server.h"
#include "tod_msgs/PVD_data.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "PVDReceiver");
    ros::NodeHandle n;
    // tod_network::Server<tod_msgs::VehicleData> receiver(n);
    // receiver.receive_processer("vehicle_data", tod_network::OperatorPorts::RX_VEHICLESTATE_VEHICLEDATA);
    // receiver.receive_run();
    tod_network::Server<tod_msgs::PVD_data> receiver(n);
    receiver.receive_processer("PVD", tod_network::OperatorPorts::RX_VEHICLE_PVD);
    receiver.receive_run();
    return 0;
}
