#pragma once
#include <ros/ros.h>
#include <tod_msgs/ControlCmd.h>
#include <mutex>
#include <string>


class rosloop_vehicle {
public:
    rosloop_vehicle() = default;
    template <class Msg> 
    void ros_run_spin_loop(const std::string& node_name, const std::string& msg_name,
                           Msg* msg,  bool* ros_terminated) {
        ros::NodeHandle nh;
        ros::Rate r(100);
        ros::Publisher pub_msg = nh.advertise<Msg>(msg_name, 1);
        while ( ros::ok() ) {
            pub_msg.publish(*msg);
            ros::spinOnce();
            r.sleep();
        }
        *ros_terminated = true;
    }

    // void ros_run_spin_loop_packet(const std::string& node_name, const std::string& msg_name,
    //                        tod_msgs::ControlCmd* msg,  bool* ros_terminated) {
    //     ros::NodeHandle nh;
    //     ros::Rate r(100);
    //     ros::Publisher pub_control_msg = nh.advertise<tod_msgs::ControlCmd>(msg_name, 1);
    //     while ( ros::ok() ) {
    //         pub_control_msg.publish(*msg);
    //         ros::spinOnce();
    //         r.sleep();
    //     }
    //     *ros_terminated = true;
    // }
private:
};
