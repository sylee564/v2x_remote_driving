#include <ros/ros.h>
#include "force_feedback_controller.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "ForceFeedbackController");
    ros::NodeHandle nh;
    ForceFeedbackController controller(nh);
    controller.run();
    return 0;
}
