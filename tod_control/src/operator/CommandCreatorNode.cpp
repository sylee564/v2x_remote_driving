#include <ros/ros.h>
#include "CommandCreator.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "CommandCreator");

    ros::NodeHandle nodeHandle;
    CommandCreator oper_command(nodeHandle);
    oper_command.run();
    return 0;
}