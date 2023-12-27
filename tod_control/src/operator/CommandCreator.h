#pragma once
#include "sensor_msgs/Joy.h"
#include "tod_msgs/Status.h"
#include "tod_msgs/ControlCmd.h"
#include "tod_msgs/VehicleEnums.h"
#include "tod_msgs/joystickConfig.h"
#include "tod_msgs/VehicleData.h"
#include <utility>
#include <stdio.h>
#include <vector>
#include <map>
#include <algorithm>
#include <ros/ros.h>
// #include <tod_core/VehicleParameters.h>

#define St_offset 5
#define MAX_SPEED 10.0
#define Oper_ID 1

using namespace std;


class CommandCreator
{
    public:
        CommandCreator(ros::NodeHandle& nodeHandle);
        ~CommandCreator() = default;
        void run();

    private:
        ros::Subscriber _joystickSubs;
        ros::Subscriber _statusSubs;
        ros::Subscriber _vehicleSubs;
        ros::Publisher _controlCmdPub;
        std::map<joystick::ButtonPos, int> _prevButtonState;
        uint8_t _status{tod_msgs::Status::TOD_STATUS_IDLE};
        uint8_t _control_type{tod_msgs::Status::CONTROL_MODE_DIRECT};
        tod_msgs::ControlCmd _controlCmdMsg;
        // std::unique_ptr<tod_core::VehicleParameters> _vehParams;
        ros::NodeHandle _nh;

        bool _constraintSteeringRate{false};
        bool _invertSteeringInGearReverse{false};
        float _maxSpeedms{10};
        float _maxAcceleration{1};
        float _maxDeceleration{3};
        double _maxSteeringWheelAngle{1.0};
        double _maxSteeringWheelAngleRate{2.618};
        bool _joystickInputSet{false};
        bool _remote_flag{false};
        bool _stream_flag{false};
        bool _avm_flag{false};

        uint8_t oper_gears{0};
        uint8_t indicator{0};
        uint32_t _operator_id{0};
        // target and current state of the wheel
        // double m_current_angle{0};
        // float m_current_brake{-1};
        // float m_current_accelerator{-1};

        uint8_t vehicle_status{0};
        float vehicle_velocity_{0.0};
        // uint8_t EPS_Control_Board_Status{0};
        // uint8_t EPS_Control_Status{0};
        // uint8_t EPS_En_Status{0};
        // uint8_t ACC_Control_Board_Status{0};
        // uint8_t ACC_Control_Status{0};
        // uint8_t ACC_En_Status{0};
        uint8_t vehicle_gear{0};
        uint8_t vehicle_gear_status{0};

        bool remote_status{false};

        void init_control_messages();
        void callback_joystick_msg(const sensor_msgs::Joy::ConstPtr &msg);
        void callback_status_msg(const tod_msgs::Status &msg);
        void callback_vehicle_data_msg(const tod_msgs::VehicleData::ConstPtr &msg);
        void calculate_desired_velocity(tod_msgs::ControlCmd &out, const sensor_msgs::Joy::ConstPtr &msg, const int gear);
        void calculate_steering_wheel_angle(tod_msgs::ControlCmd& out, const std::vector<float>& axes); 

        void set_gear(tod_msgs::ControlCmd &out, const std::vector<int> &buttonState,
            const float &currentVelocity);
        void set_indicator(tod_msgs::ControlCmd &out, const std::vector<int> &buttonState);
        void set_remote(tod_msgs::ControlCmd &out, const std::vector<int> &buttonState, const float &currentVelocity);
        void set_video_stream(tod_msgs::ControlCmd &out, const std::vector<int> &buttonState, const float &currentVelocity);
        // void set_emergency_stop(tod_msgs::_controlCmdMsg &out, const std::vector<int> &buttonState);
};  