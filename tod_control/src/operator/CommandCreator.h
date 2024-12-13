#pragma once
#include "sensor_msgs/Joy.h"
#include "tod_msgs/Status.h"
#include "tod_msgs/ControlCmd.h"
#include "tod_msgs/V2XDataBase.h"
#include "tod_msgs/VehicleEnums.h"
#include "tod_msgs/joystickConfig.h"
#include "tod_msgs/ProbeVehicleData.h"
#include <utility>
#include <stdio.h>
#include <vector>
#include <map>
#include <algorithm>
#include <ros/ros.h>
#include <tod_helper/vehicle/Model.h>
// #include <tod_core/VehicleParameters.h>

#define St_offset 5
#define MAX_SPEED 10.0
#define Oper_ID "Kona_oper"

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
        tod_msgs::V2XDataBase _v2xDB;
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
        // std::string _operator_id;
        int _operator_id;
        bool _joystickInputSet{false};

        uint8_t _remote_flag{0};
        uint8_t _stream_flag{0};
        bool _avm_flag{false};
        bool _parking_flag{false};
        uint8_t _aeb_flag{0};

        uint8_t _steering_mode{0};
        uint8_t _drive_mode{0};

        uint8_t oper_gears{0};
        uint8_t indicator{0};

        uint8_t vehicle_status{0};
        float vehicle_velocity_{0.0};
        uint8_t vehicle_gear{0};

        void init_control_messages();
        double map(double output, double in_min, double in_max, double out_min, double out_max);
        void callback_joystick_msg(const sensor_msgs::Joy::ConstPtr &msg);
        void callback_status_msg(const tod_msgs::Status &msg);
        void callback_vehicle_data_msg(const tod_msgs::ProbeVehicleData::ConstPtr &msg);
        void raw_longitudinal_control(tod_msgs::ControlCmd &out, const sensor_msgs::Joy::ConstPtr &msg);
        void calculate_desired_velocity(tod_msgs::ControlCmd &out, const sensor_msgs::Joy::ConstPtr &msg);
        void calculate_steering_wheel_angle(tod_msgs::ControlCmd& out, const std::vector<float>& axes); 

        void set_gear(tod_msgs::ControlCmd &out, const std::vector<int> &buttonState,
            const float &currentVelocity);
        void set_indicator(tod_msgs::ControlCmd &out, const std::vector<int> &buttonState);
        void set_remote(tod_msgs::ControlCmd &out, const std::vector<int> &buttonState, const float &currentVelocity);
        void set_video_stream(tod_msgs::ControlCmd &out, const std::vector<int> &buttonState, const float &currentVelocity);
        // void set_avm(tod_msgs::ControlCmd &out, const std::vector<int> &buttonState, const float &currentVelocity);
        void set_emergency_stop(tod_msgs::ControlCmd &out, const std::vector<int> &buttonState);
        // void set_parking_brake(tod_msgs::ControlCmd &out, const std::vector<int> &buttonState, const float &currentVelocity);
        void set_steer_mode(tod_msgs::ControlCmd &out, const std::vector<int> &buttonState, const float &currentVelocity);
        void set_drive_mode(tod_msgs::ControlCmd &out, const std::vector<int> &buttonState, const float &currentVelocity);
};  