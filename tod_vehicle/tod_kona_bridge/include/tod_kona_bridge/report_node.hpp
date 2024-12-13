#ifndef PIX_DRIVER_HPP
#define PIX_DRIVER_HPP

#include "ros/ros.h"
#include "steering_report_502.hpp"
#include "can_msgs/Frame.h"
#include "pix_driver_msgs/SteerReport.h"

namespace pix_driver
{
class PixDriverReport
{
    public:
        PixDriverReport();
        void update_steering();
        static void can_msg_callback(const can_msgs::Frame &msg);
        void run();

    private:
        ros::NodeHandle nh_;
        ros::Subscriber can_sub_;
        ros::Publisher steer_pub_;
        Steeringreport502 steer_report;
        pix_driver_msgs::SteerReport steer_report_msg;
        static can_msgs::Frame can_frame_msg;
};

}

#endif