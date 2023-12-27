#include "RosInterface.h"
#include <ros/package.h>
#include "InputDeviceController.h"
#include <QMainWindow>

RosInterface::RosInterface(int argc, char** pArgv, InputDeviceController* parent)
    : m_Init_argc(argc), m_pInit_argv(pArgv), _parent(parent) {

    clear_joystick_msg();
}

RosInterface::~RosInterface() {
    _rosThread.join();
}

void RosInterface::set_debug_mode() {
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
        ros::console::notifyLoggerLevelsChanged();
}

bool RosInterface::init() {
    ros::init(m_Init_argc, m_pInit_argv, "InputDevice");
    nh = std::make_unique<ros::NodeHandle>();
    if (!ros::master::check())
        return false;//do not start without ros.

    bool debug{false};
    nh->getParam(ros::this_node::getName() + "/debug", debug);
    if (debug) // print ROS_DEBUG
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
            ros::console::notifyLoggerLevelsChanged();

    ros::start();
    ros::Time::init();
    _joystickMsgPublisher  = nh->advertise<sensor_msgs::Joy>("joystick", 100);
    _rosThread = std::thread([=]{ run();});

    return true;
}

bool RosInterface::clear_joystick_msg() {
    _operatorMsg.buttons.assign(31, 0);
    _operatorMsg.axes.assign(4, 0);
    return true;
}

void RosInterface::run() {
    ros::Rate r(1000);

    while ( ros::ok() && _active ) {
        _operatorMsg.header.stamp = ros::Time::now();
        _joystickMsgPublisher.publish(_operatorMsg);
        ros::spinOnce();
        r.sleep();
    }
    if ( _parent != nullptr ) {
        _parent->terminate();
    }
}

std::string RosInterface::get_node_name() {
    return ros::this_node::getName();
}

std::string RosInterface::get_package_path() {
    return ros::package::getPath("tod_input_devices");
}

void RosInterface::set_button(const int &button, const int &state) {
    if ( button > _operatorMsg.buttons.size() - 1 ) {
        ROS_ERROR_STREAM(get_node_name() << ": tried to access button " << button
            << " only buttons between 0 and " << _operatorMsg.buttons.size() - 1
            << " possible!");
        return;
    }
    _operatorMsg.buttons.at(button) = state;
}

void RosInterface::set_axis(const int &axis, const float &value) {
    if ( axis > _operatorMsg.axes.size() - 1 ) {
        ROS_ERROR_STREAM(get_node_name() << ": tried to access axis " << axis
            << " only axes between 0 and " << _operatorMsg.axes.size() - 1
            << " possible!");
        return;
    }
    _operatorMsg.axes.at(axis) = value;
}

