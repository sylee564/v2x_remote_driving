#pragma once
#include <ros/ros.h>
#include <tod_msgs/Status.h>
#include <tod_msgs/ControlCmd.h>
#include <mutex>
#include <string>

class rosloop_operator {
public:
    tod_msgs::ControlCmd controlCmdStatus;

    bool gotNewStudyLeaderWishes(tod_msgs::Status& outputLeaderWishes) {
        if ( gotSomethingNew ) {
            std::lock_guard<std::mutex> lock(mutexForStudyLeaderWishes);
            gotSomethingNew = false;
            outputLeaderWishes = studyLeaderWishes;
            return true;
        } else {
            return false;
        }
    }

    rosloop_operator() = default;
    void ros_run_spin_loop(const std::string& node_name, const std::string& msg_name,
                           tod_msgs::Status* msg, bool* ros_terminated) {
        ros::NodeHandle nh;
        ros::Rate r(100);
        ros::Publisher pub_status_msg = nh.advertise<tod_msgs::Status>(msg_name, 1);
        ros::Subscriber subToStudyLeaderWishes = nh.subscribe("study_leader_status", 5,
            &rosloop_operator::callbackStudyLeaderWishes, this);
        // ros::Subscriber subToControlCmd = nh.subscribe("/Operator/CommandCreation/control_cmd_data", 5,
        //     &rosloop_operator::callbackControlCmd, this);
        while ( ros::ok() ) {
            pub_status_msg.publish(*msg);
            ros::spinOnce();
            r.sleep();
        }
        *ros_terminated = true;
    }

private:
    bool gotSomethingNew{ false };
    tod_msgs::Status studyLeaderWishes;
    
    std::mutex mutexForStudyLeaderWishes;

    void callbackStudyLeaderWishes(const tod_msgs::Status& status) {
        std::lock_guard<std::mutex> lock(mutexForStudyLeaderWishes);
        studyLeaderWishes = status;
        gotSomethingNew = true;
    }
};

class rosloop_vehicle {
public:
    rosloop_vehicle() = default;
    void ros_run_spin_loop(const std::string& node_name, const std::string& msg_name,
                           tod_msgs::Status* msg,  bool* ros_terminated) {
        ros::NodeHandle nh;
        ros::Rate r(100);
        ros::Publisher pub_status_msg = nh.advertise<tod_msgs::Status>(msg_name, 1);
        while ( ros::ok() ) {
            pub_status_msg.publish(*msg);
            ros::spinOnce();
            r.sleep();
        }
        *ros_terminated = true;
    }
private:
};
