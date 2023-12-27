#include "tod_network/tod_server.h"
#include "tod_msgs/ControlCmd.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "ControlCommandSender");
    ros::NodeHandle n;
    tod_network::Server<tod_msgs::ControlCmd> sender(n, false);
   
    sender.send_processer("control_cmd_data", tod_network::OperatorPorts::TX_CONTROL_COMMAND);

    sender.send_run();
    return 0;
}
