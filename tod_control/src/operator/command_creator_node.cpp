#include "command_creator.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "OperatorCommandCreator");
  ros::NodeHandle nh;

  tod_control::CommandCreator node(nh);
  node.run();
  return 0;
}
