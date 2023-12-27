#include <ros/ros.h>
#include "tod_network/tod_server.h"
#include "tod_msgs/VehicleData.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "VehicleDataReceiver");
    ros::NodeHandle n;

    tod_network::Server<tod_msgs::VehicleData> receiver(n);
    receiver.receive_processer("vehicle_data", tod_network::OperatorPorts::RX_VEHICLESTATE_VEHICLEDATA);
    receiver.receive_run();
    return 0;
}
