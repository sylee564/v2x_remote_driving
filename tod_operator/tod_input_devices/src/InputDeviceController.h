#pragma once
#include <stdio.h>
#include <ros/ros.h>
#include <map>
#include <string>
#include <memory>
#include <utility>
#include "VirtualInputDevice.h"
#include "UsbInputDevice.h"
#include "RosInterface.h"
#include "tod_msgs/joystickConfig.h"
#include "tod_msgs/InputDevice.h"
#include "sensor_msgs/Joy.h"
#include <filesystem>

struct AxisItem{
    AxisItem(joystick::AxesPos pos, bool inv): position(pos), invertAxes(inv) {}
    joystick::AxesPos position;
    bool invertAxes;
};

class InputDeviceController {
public:
    explicit InputDeviceController(int argc, char** argv);
    ~InputDeviceController();
    void terminate();

private:
    // Input Device Configuration
    std::map<std::string, std::shared_ptr<InputDevice>> _inputDevices;
    std::map<int, joystick::ButtonPos> _buttonMapping;
    std::map<int, AxisItem> _axisMapping;

    // Input Devices Callback
    void callback_axis_changed(const int axis, const double value);
    void callback_button_changed(const int button, const int state);
    void callback_error(const std::string& errorMsg);

    // Service Callback
    bool callback_change_device_request(tod_msgs::InputDevice::Request& req, tod_msgs::InputDevice::Response& res);

    // other Member Variables
    bool _configurationMode{false};
    std::shared_ptr<RosInterface> _ros;
    
    // other Member Functions
    void change_input_device();
    void update_mapping_from_param_workspace();
};
