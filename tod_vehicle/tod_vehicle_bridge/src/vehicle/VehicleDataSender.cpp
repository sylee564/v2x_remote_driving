#include "ros/ros.h"
#include "tod_network/tod_client.h"
#include "tod_msgs/VehicleData.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "VehicleDataSender");
    ros::NodeHandle n;
    // tod_network::Sender<tod_msgs::VehicleData> sender(n, true);
    // sender.add_processer("vehicle_data", tod_network::OperatorPorts::RX_VEHICLESTATE_VEHICLEDATA);
    // sender.run();
    tod_network::Client<tod_msgs::VehicleData> sender(n, true);
    sender.send_processer("vehicle_data", tod_network::OperatorPorts::RX_VEHICLESTATE_VEHICLEDATA);
    sender.send_run();
    return 0;
}
