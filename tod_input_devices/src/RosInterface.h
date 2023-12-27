#pragma once
#include <thread>
#include <memory>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <ros/console.h>
#include "sensor_msgs/Joy.h"

class InputDeviceController;

class RosInterface{
public:
    explicit RosInterface(int argc, char **pArgv, InputDeviceController* parent);
    ~RosInterface();
    bool init();
    void terminate() { _active = false;}
    bool has_param(const std::string& paramName) {return nh->hasParam(paramName);}
    bool clear_joystick_msg();

    // Getters
    static std::string get_node_name();
    static std::string get_package_path();
    
    // Setters
    void set_axis(const int &axis, const float &value);
    void set_button(const int &button, const int &state);
    void set_debug_mode();

    template <typename T, typename TF, typename... Ts>
    void add_subscriber(
        const std::string& topicName, TF&& func , Ts... args) {
        if ( nh )
            _subscriber.push_back(nh->subscribe<T>(topicName, 1, boost::bind(func, _1, args...)));
    }

    template<typename T>
    bool get_param(const std::string& paramName, T &retVal) {
        if (!nh->getParam(paramName, retVal)) {
            ROS_ERROR("%s: could not get param %s", this->get_node_name().c_str(),
                paramName.c_str());
            return false;
        }
        return true;
    }
    template<typename T>
    bool get_optional_param(const std::string& paramName, T &retVal) {
        if (!nh->getParam(paramName, retVal)) {
            ROS_DEBUG("%s: could not get param %s", this->get_node_name().c_str(),
                paramName.c_str());
            return false;
        }
        return true;
    }

    std::unique_ptr<ros::NodeHandle> nh;
    std::vector<ros::ServiceServer> services;

private:
    int m_Init_argc;
    char** m_pInit_argv;
    bool _active{true};
    std::thread _rosThread;
    std::vector<ros::Subscriber> _subscriber;
    ros::Publisher  _joystickMsgPublisher;
    sensor_msgs::Joy _operatorMsg;
    InputDeviceController* _parent;
    void run();
};
