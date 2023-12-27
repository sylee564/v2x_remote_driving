#include <ros/ros.h>
#include "tod_network/tod_client.h"
#include "tod_msgs/ControlCmd.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "OperatorNetwork");
    ros::NodeHandle n;
    tod_network::Client<tod_msgs::ControlCmd> client(n, false);
    client.client_processer("control_cmd_data");
    // client.send_processer("control_cmd");
    // client.send_in_control_mode(tod_msgs::Status::CONTROL_MODE_DIRECT);
    // receiver.send_in_control_mode(tod_msgs::Status::CONTROL_MODE_INDIRECT);
    client.tod_client_run();
    return 0;
}
