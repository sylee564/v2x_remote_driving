#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "UsbEventDevice/UsbEventDevice.h"

class UsbEventHandler {
public:
    explicit UsbEventHandler(ros::NodeHandle &nh)
        : _nh{nh}, _joystick{"/dev/input/event"} {
        _sub = _nh.subscribe<std_msgs::Float64>(
            "/Operator/CommandCreation/force_feedback", 5, &UsbEventHandler::callback_force_feedback, this);
    }
    ~UsbEventHandler() {}
    void run() {
        ros::Rate r(20);
        while (ros::ok() && _joystick.ok()) {
            r.sleep();
            ros::spinOnce();
            static ros::Time resetTime{ros::Time::now()};
            if (ros::Time::now() >= resetTime + ros::Duration(25.0)) {
                if (_joystick.ok()) _joystick.reset();
                resetTime = ros::Time::now();
            }
        }
    }

private:
    ros::NodeHandle& _nh;
    UsbEventDevice _joystick;
    ros::Subscriber _sub;

    void callback_force_feedback(const std_msgs::Float64ConstPtr &msg) {
        if (_joystick.ok()) {
            _joystick.set_force_feedback(msg->data);
        } else {
            ROS_WARN_ONCE("%s: joystick not ok - not setting force feedback",
                          ros::this_node::getName().c_str());
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "UsbEventDevice");
    ros::NodeHandle nh;
    UsbEventHandler node(nh);
    node.run();
    return 0;
}

