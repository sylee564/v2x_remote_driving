#include "tod_network/tod_client.h"
#include "tod_msgs/ControlCmd.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "ControlCommandReceiver");
    ros::NodeHandle n;
    tod_network::Client<tod_msgs::ControlCmd> receiver(n, true);
    receiver.receive_processer("control_cmd_data", tod_network::OperatorPorts::TX_CONTROL_COMMAND);
    // receiver.send_in_control_mode(tod_msgs::Status::CONTROL_MODE_DIRECT);
    // receiver.send_in_control_mode(tod_msgs::Status::CONTROL_MODE_INDIRECT);
    receiver.receive_run();
    return 0;
}
