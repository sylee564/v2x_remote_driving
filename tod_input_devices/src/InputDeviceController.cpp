#include "InputDeviceController.h"
#include <stdlib.h>
#include <functional>

InputDeviceController::InputDeviceController(int argc, char **argv) {
    _ros = std::make_unique<RosInterface>(argc, argv, this);
    _ros->init();
    _ros->services.push_back(_ros->nh->advertiseService(_ros->get_node_name()+"/change_input_device",
        &InputDeviceController::callback_change_device_request, this));

    bool debug{false};
    _ros->get_param(_ros->get_node_name() + "/debug", debug);
    if (debug)
        _ros->set_debug_mode();

    auto axisCallback = std::bind(
        &InputDeviceController::callback_axis_changed, this, std::placeholders::_1, std::placeholders::_2);
    auto buttonCallback = std::bind(
        &InputDeviceController::callback_button_changed, this, std::placeholders::_1, std::placeholders::_2);
    auto errorCallback = std::bind(
        &InputDeviceController::callback_error, this, std::placeholders::_1);

    _inputDevices["Virtual"] = std::make_shared<VirtualInputDevice>(axisCallback, buttonCallback);
    _inputDevices["Usb"] = std::make_shared<UsbInputDevice>(axisCallback, buttonCallback, errorCallback);
    change_input_device(); //Default Parameter-File loaded in launch-File
}

InputDeviceController::~InputDeviceController() {
    _ros->terminate();
}

void InputDeviceController::terminate() {
    for ( auto& device : _inputDevices ) { // Deactivate others
        ROS_DEBUG_STREAM(_ros->get_node_name() << ": Deactivating Input Device: " << device.first);
        device.second->terminate();
    }
}

void InputDeviceController::callback_axis_changed(const int axis, const double value) {
    if ( _configurationMode )
        ROS_INFO_STREAM("Axis:" << axis << ", Value: " << value);

    int sign;
    if ( _axisMapping.count(axis) != 0 ) {
        _axisMapping.at(axis).invertAxes ? sign = -1 : sign = 1;
        _ros->set_axis(_axisMapping.at(axis).position, sign*value); //sets Button at MsgPos with HW Button as input
    }
}

void InputDeviceController::callback_button_changed(const int button, const int state) {
    if ( _configurationMode )
        ROS_INFO_STREAM("Button:" << button << ", State: " << state);

    if ( _buttonMapping.count(button) != 0 )
        _ros->set_button(_buttonMapping.at(button), state); //sets Button at MsgPos with HW Button as input
}

void InputDeviceController::callback_error(const std::string& errorMsg) {
    ROS_ERROR_STREAM_ONCE(errorMsg);
    _ros->terminate();
}

bool InputDeviceController::callback_change_device_request(
    tod_msgs::InputDevice::Request& req, tod_msgs::InputDevice::Response& res) {

    std::string cfg_file_path = req.input_device_directory;
    std::string load_command = "rosparam load " + cfg_file_path + " " + _ros->get_node_name();
    namespace fsys = std::filesystem;
    if ( !fsys::exists(cfg_file_path) || !(fsys::is_regular_file(cfg_file_path) || fsys::is_symlink(cfg_file_path)) ) {
        ROS_ERROR_STREAM(_ros->get_node_name() <<": Could not find or open config file " << cfg_file_path);
        res.successfully_changed = false;
        return false;
    } else {
        ROS_DEBUG_STREAM(_ros->get_node_name() << ": Executing command: " << load_command);
        int result = std::system(load_command.c_str());
        if ( result != 0 ) {
            res.successfully_changed = false;
            ROS_ERROR_STREAM(_ros->get_node_name() <<": Could not load config file " << cfg_file_path);
            return false;
        }
    }

    change_input_device();
    res.successfully_changed = true;
    return true;
}

void InputDeviceController::change_input_device() {
    std::string inputDeviceType;
    _ros->get_param(_ros->get_node_name()+"/Type", inputDeviceType);
    ROS_DEBUG_STREAM(_ros->get_node_name() << ": Try changing to Type: " << inputDeviceType);

    // Deactivate all input devices
    for ( auto& device : _inputDevices ) {
        ROS_DEBUG_STREAM(_ros->get_node_name() << ": Input Device " << device.first
            << (device.second->running ? " running" : " not running"));
        if ( device.second->running ) { // if running
            ROS_DEBUG_STREAM(_ros->get_node_name() << ": Deactivating Input Device: " << device.first);
            device.second->deactivate();
        }
    }
    // Clear
    _ros->clear_joystick_msg();

    // Load new parameters
    update_mapping_from_param_workspace();

    std::string _correction;
    if (_ros->get_optional_param(_ros->get_node_name() + "/Correction", _correction))
        _inputDevices[inputDeviceType]->set_correction(_correction);

    // Start desired input device
    if ( !_inputDevices[inputDeviceType]->running ) {
         ROS_DEBUG_STREAM(_ros->get_node_name() << ": Activating Input Device: " << inputDeviceType);
        _inputDevices[inputDeviceType]->activate();
        ROS_DEBUG_STREAM(_ros->get_node_name() << ": Input Device: " << inputDeviceType << " activated");
    }
    if (_configurationMode) {
        ROS_INFO_STREAM("Number Of Axes:" << _inputDevices[inputDeviceType]->get_number_of_axes());
        ROS_INFO_STREAM("Number Of buttons:" << _inputDevices[inputDeviceType]->get_number_of_buttons());
    }
}
void InputDeviceController::update_mapping_from_param_workspace() {
    _buttonMapping.clear();
    _axisMapping.clear();
    std::map<std::string, int> buttonConfig;
    _ros->get_param(_ros->get_node_name() + "/ButtonConfig/", buttonConfig);

    for ( auto conf : buttonConfig ) {
        if ( conf.first == "IndicatorLeft" )   {_buttonMapping[conf.second] = joystick::ButtonPos::INDICATOR_LEFT;}
        if ( conf.first == "IndicatorRight" )  {_buttonMapping[conf.second] = joystick::ButtonPos::INDICATOR_RIGHT;}
        if ( conf.first == "EmergencyLight" )      {_buttonMapping[conf.second] = joystick::ButtonPos::EMERGENCY_LIGHT;}
        if ( conf.first == "IncreaseSpeed" )   {_buttonMapping[conf.second] = joystick::ButtonPos::INCREASE_SPEED;}
        if ( conf.first == "DecreaseSpeed" )   {_buttonMapping[conf.second] = joystick::ButtonPos::DECREASE_SPEED;}
        if ( conf.first == "Drive" )      {_buttonMapping[conf.second] = joystick::ButtonPos::DRIVE;}
        if ( conf.first == "Reverse" )            {_buttonMapping[conf.second] = joystick::ButtonPos::REVERSE;}
        if ( conf.first == "Neutral" )   {_buttonMapping[conf.second] = joystick::ButtonPos::NEUTRAL;}
        if ( conf.first == "Park" )   {_buttonMapping[conf.second] = joystick::ButtonPos::PARK;}
        if ( conf.first == "Remote" )   {_buttonMapping[conf.second] = joystick::ButtonPos::REMOTE;}
        if ( conf.first == "VideoStream" )   {_buttonMapping[conf.second] = joystick::ButtonPos::VIDEO;}
        if ( conf.first == "AVMStream" )   {_buttonMapping[conf.second] = joystick::ButtonPos::AVM;}
    }

    std::map<std::string, bool> invertAxisConfig;
    std::map<std::string, int> axisConfig;
    _ros->get_param(_ros->get_node_name() + "/InvertAxis/", invertAxisConfig);
    _ros->get_param(_ros->get_node_name() + "/AxisConfig/", axisConfig);
    _ros->get_param(_ros->get_node_name() + "/ConfigMode/", _configurationMode);

    for ( auto conf : axisConfig ) {
        if ( conf.first == "Steering" ) {
            _axisMapping.insert(std::make_pair(
                conf.second, AxisItem(joystick::AxesPos::STEERING, invertAxisConfig.at("Steering"))));
        }
        if ( conf.first == "Throttle" ) {
            _axisMapping.insert(
                std::make_pair(conf.second, AxisItem(joystick::AxesPos::THROTTLE, invertAxisConfig.at("Throttle"))));
        }
        if ( conf.first == "Brake" ) {
            _axisMapping.insert(
                std::make_pair(conf.second, AxisItem(joystick::AxesPos::BRAKE, invertAxisConfig.at("Brake"))));
        }
    }
}
