#include "CommandCreator.h"


CommandCreator::CommandCreator(ros::NodeHandle& nodeHandle) :_nh(nodeHandle)
{
    /*ROS node create*/
    _joystickSubs = nodeHandle.subscribe("/Operator/InputDevices/joystick", 1,
                                         &CommandCreator::callback_joystick_msg, this);
    _statusSubs = nodeHandle.subscribe("/Operator/Manager/status_msg", 1,
                                       &CommandCreator::callback_status_msg, this);
    _vehicleSubs = nodeHandle.subscribe("/Operator/kona/probe_vehicle_data", 1,
                                       &CommandCreator::callback_vehicle_data_msg, this);
    _controlCmdPub = nodeHandle.advertise<tod_msgs::ControlCmd>("control_cmd_data", 1);

    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::INDICATOR_LEFT, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::INDICATOR_RIGHT, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::EMERGENCY_SIGNAL, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::INCREASE_SPEED, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::DECREASE_SPEED, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::PARK, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::REVERSE, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::NEUTRAL, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::DRIVE, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::REMOTE, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::VIDEO, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::AVM, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::AEB_FLAG, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::PARK_FLAG, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::DRIVE_MODE, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::STEER_MODE, 0));

    if (!nodeHandle.getParam(ros::this_node::getName() + "/ConstraintSteeringRate", _constraintSteeringRate))
        ROS_ERROR_STREAM(ros::this_node::getName() << ": Could not get param /ConstraintSteeringRate - using "
                                                   << (_constraintSteeringRate ? "true" : "false"));    

    if (!nodeHandle.getParam(ros::this_node::getName() + "/maxVelocity", _maxSpeedms))
        ROS_ERROR_STREAM(ros::this_node::getName() << ": Could not get param /maxVelocity - using "
                                                   << _maxSpeedms << " m/s");

    if (!nodeHandle.getParam(ros::this_node::getName() + "/maxAcceleration", _maxAcceleration))
        ROS_ERROR_STREAM(ros::this_node::getName() << ": Could not get param /maxAcceleration - using "
                                                   << _maxAcceleration << " m/s^2");

    if (!nodeHandle.getParam(ros::this_node::getName() + "/maxDeceleration", _maxDeceleration))
        ROS_ERROR_STREAM(ros::this_node::getName() << ": Could not get param /maxDeceleration - using "
                                                   << _maxDeceleration << " m/s^2");

    if (!nodeHandle.getParam(ros::this_node::getName() + "/maxSteeringWheelAngleRate", _maxSteeringWheelAngleRate))
    ROS_ERROR_STREAM(ros::this_node::getName() << ": Could not get param /maxSteeringWheelAngleRate - using "
                                                << _maxSteeringWheelAngleRate << " rad/s");
    
    if (!nodeHandle.getParam(ros::this_node::getName() + "/OperatorID", _operator_id))
    ROS_ERROR_STREAM(ros::this_node::getName() << ": Could not get param /OperatorID - using "<< _operator_id );
    
                                                   
    ros::Duration(1).sleep();
    ROS_WARN("Please Wait.................................");
}

void CommandCreator::run() {
    ros::Rate loop_rate(50);
    while (ros::ok()) {
        ros::spinOnce();
        // if (_vehParams->vehicle_id_has_changed()) {
        //     _vehParams->load_parameters();
        // }
        // if(1){
        
        if (_joystickInputSet && _status == tod_msgs::Status::TOD_STATUS_TELEOPERATION) {
            // _controlCmdMsg.header.stamp = ros::Time::now();
            // _controlCmdMsg.header.frame_id = "operator_command";
            _controlCmdMsg.operator_id = _operator_id;
            _controlCmdPub.publish(_controlCmdMsg);
        }
        _joystickInputSet = false;
        loop_rate.sleep();
    }
}

void CommandCreator::callback_joystick_msg(const sensor_msgs::Joy::ConstPtr& msg) {
    // if (1) {
    if (_status == tod_msgs::Status::TOD_STATUS_TELEOPERATION) {
        set_remote(_controlCmdMsg, msg->buttons, vehicle_velocity_);
        set_video_stream(_controlCmdMsg, msg->buttons, vehicle_velocity_);
        // set_avm(_controlCmdMsg, msg->buttons, vehicle_velocity_);
        // set_parking_brake(_controlCmdMsg, msg->buttons, vehicle_velocity_);

        _controlCmdMsg.control_type = _control_type;
        _controlCmdMsg.v2x_db = _v2xDB;
        if(_remote_flag){
            calculate_steering_wheel_angle(_controlCmdMsg, msg->axes);
            raw_longitudinal_control(_controlCmdMsg, msg);
            calculate_desired_velocity(_controlCmdMsg, msg);
            set_gear(_controlCmdMsg, msg->buttons, vehicle_velocity_);
            set_indicator(_controlCmdMsg, msg->buttons);
            set_emergency_stop(_controlCmdMsg, msg->buttons);
            set_steer_mode(_controlCmdMsg, msg->buttons, vehicle_velocity_);
            set_drive_mode(_controlCmdMsg, msg->buttons, vehicle_velocity_);
            // set_light(_secondaryControlMsg, msg->buttons);
        }
        else{
            init_control_messages();
        }
        _joystickInputSet = true;
    }
}

void CommandCreator::callback_vehicle_data_msg(const tod_msgs::ProbeVehicleData::ConstPtr &msg) {
    vehicle_velocity_ = msg->velocity;
    vehicle_gear = msg->gear_status;
}

void CommandCreator::calculate_steering_wheel_angle(tod_msgs::ControlCmd& out,
        const std::vector<float>& axes) {
    static ros::Time tPrev;
    static ros::Duration dur;
    static double oldSetSWA{0.0};
    static double steeringRate{0.0};
    // calc desired SWA
    // double newDesiredSWA = axes.at(joystick::AxesPos::STEERING);
    double newDesiredSWA = -axes.at(joystick::AxesPos::STEERING) * 7.8538;
     if (_constraintSteeringRate) { // constraint steering rate
        dur = ros::Time::now() - tPrev;
        double newSetSWA = std::min(newDesiredSWA, oldSetSWA + dur.toSec() * _maxSteeringWheelAngleRate);
        newSetSWA = std::max(newSetSWA, oldSetSWA - dur.toSec() * _maxSteeringWheelAngleRate);
        oldSetSWA = newSetSWA;
        out.control.steering_angle = newSetSWA;
        tPrev = ros::Time::now();
    } else { // output unconstraint SWA
        dur = ros::Time::now() - tPrev;
        steeringRate = ((newDesiredSWA-oldSetSWA)/(dur.toSec()));
        out.control.steering_angle_velocity = abs(steeringRate);
        oldSetSWA = newDesiredSWA;
        tPrev = ros::Time::now();
        out.control.steering_angle = newDesiredSWA;
    }

    if(out.indicator.data == 1 ){
        if(out.control.steering_angle* 180.0 / 3.1415 > 70){
            if(steeringRate<-0.5) out.indicator.data=0;
        }
    }
    else if(out.indicator.data == 2){
        if(out.control.steering_angle* 180.0 / 3.1415 < -70){
            if(steeringRate>0.5) out.indicator.data=0;
        }
    }

    // out.control.steering_angle =  -newDesiredSWA*7.8538;  //maximum steering wheel angle radian 

    // if (_invertSteeringInGearReverse &&
    //     _secondaryControlMsg.gearPosition == eGearPosition::GEARPOSITION_REVERSE)
    //     out.steeringWheelAngle = -out.steeringWheelAngle;

}

void CommandCreator::raw_longitudinal_control(tod_msgs::ControlCmd &out, const sensor_msgs::Joy::ConstPtr &msg){
    
    if (vehicle_gear == eGearPosition::GEARPOSITION_NEUTRAL || vehicle_gear == eGearPosition::GEARPOSITION_PARK) {
        out.control.throttle = 0.0;
        out.control.brake = 0.0;
        return;
    }

    // read Param Server
    bool inputDeviceHasSeparateBrakingAxis{true};
    if (!ros::param::get("/Operator/InputDevices/InputDevice/InputDeviceHasSeparateBrakingAxis/",
                         inputDeviceHasSeparateBrakingAxis)) {
        ROS_ERROR_ONCE("'/Operator/InputDevices/InputDevice/InputDeviceHasSeparateBrakingAxis/' was not set. "
                       "StandardMode is used!");
    }
    if(!inputDeviceHasSeparateBrakingAxis){
        if(msg->axes.at(joystick::AxesPos::THROTTLE)>0){
            out.control.throttle = map(msg->axes.at(joystick::AxesPos::THROTTLE), 0.0, 1.0, 0.0, 100.0);
            out.control.brake = 0.0;
        }
        else{
            out.control.throttle = 0.0;
            out.control.brake = map(msg->axes.at(joystick::AxesPos::THROTTLE), 0.0, -1.0, 0.0, 100.0);
        }

    }
    else{
        if (vehicle_velocity_ >= _maxSpeedms*3.6) {
            out.control.throttle = 0.0; //Limit demanded speed
            out.control.brake = map(msg->axes.at(joystick::AxesPos::BRAKE), -1.0, 1.0, 0.0, 100.0);
        }else{
            out.control.throttle = map(msg->axes.at(joystick::AxesPos::THROTTLE), -1.0, 1.0, 0.0, 100.0);
            out.control.brake = map(msg->axes.at(joystick::AxesPos::BRAKE), -1.0, 1.0, 0.0, 100.0);
        }
    }

}

void CommandCreator::calculate_desired_velocity(tod_msgs::ControlCmd &out,
        const sensor_msgs::Joy::ConstPtr &msg)
{   
    if (vehicle_gear == eGearPosition::GEARPOSITION_PARK || vehicle_gear == eGearPosition::GEARPOSITION_NEUTRAL) {
        out.velocity = 0.0;
        out.acceleration = 0.0;
        return;
    }

    static ros::Time prevTime = ros::Time::now();
    float a_soll = 0.0;
    float changeOperator;

    // read Param Server
    bool inputDeviceHasSeparateBrakingAxis{true};
    if (!ros::param::get("/Operator/InputDevices/InputDevice/InputDeviceHasSeparateBrakingAxis/",
                         inputDeviceHasSeparateBrakingAxis)) {
        ROS_ERROR_ONCE("'/Operator/InputDevices/InputDevice/InputDeviceHasSeparateBrakingAxis/' was not set. "
                       "StandardMode is used!");
    }
    // printf("Separated Braking Axis : %d\n",inputDeviceHasSeparateBrakingAxis);
    if (!inputDeviceHasSeparateBrakingAxis) {
        changeOperator = msg->axes.at(joystick::AxesPos::THROTTLE);
    }
    else{
        changeOperator = (msg->axes.at(joystick::AxesPos::THROTTLE) - msg->axes.at(joystick::AxesPos::BRAKE)) / 2.0;
    }
    
    //Calculate Acceleration demand by operator
    static float deadzoneThrottle{0.05}, deadzoneBrake{0.05};
    if (changeOperator >= 0) {
        a_soll = _maxAcceleration * (std::max(changeOperator, (float) deadzoneThrottle) - deadzoneThrottle); //acc
    } else {
        a_soll = _maxDeceleration * (std::min(changeOperator, (float) -deadzoneBrake) + deadzoneBrake); // decelerate
    }

    //Integrate Speed
    ros::Duration dt = ros::Time::now() - prevTime;
    prevTime = ros::Time::now();
    out.velocity = out.velocity + dt.toSec() * a_soll;


    // Handle Speed Button Increase/Decrease
    if (msg->buttons.at(joystick::ButtonPos::INCREASE_SPEED) == 1
        && _prevButtonState.at(joystick::ButtonPos::INCREASE_SPEED) == 0) {
        out.velocity += 1.0 / 3.6; // kmh increments
    }
    if (msg->buttons.at(joystick::ButtonPos::DECREASE_SPEED) == 1
        && _prevButtonState.at(joystick::ButtonPos::DECREASE_SPEED) == 0) {
        out.velocity -= 1.0 / 3.6; // kmh increments
    }
    
    _prevButtonState.at(joystick::ButtonPos::INCREASE_SPEED) = msg->buttons.at(joystick::ButtonPos::INCREASE_SPEED);
    _prevButtonState.at(joystick::ButtonPos::DECREASE_SPEED) = msg->buttons.at(joystick::ButtonPos::DECREASE_SPEED);
        
    // Saturate Speed integration and limit Acceleration
    if (vehicle_velocity_ >= _maxSpeedms) {
        out.velocity = _maxSpeedms; //Limit demanded speed
    } else if (out.velocity < 0) {
        out.velocity = 0; //Limit demanded speed to zero
    }

    out.acceleration = a_soll;
}

void CommandCreator::callback_status_msg(const tod_msgs::Status &msg) {
    if (_status == tod_msgs::Status::TOD_STATUS_TELEOPERATION
        && msg.tod_status != tod_msgs::Status::TOD_STATUS_TELEOPERATION) {
        init_control_messages();
        // printf("initialization");
    }
    _v2xDB = msg.v2x_db;
    _control_type = msg.control_mode;
    _status = msg.tod_status;
    
}

void CommandCreator::set_gear(tod_msgs::ControlCmd &out, const std::vector<int> &buttonState,
        const float &currentVelocity) {

    if (currentVelocity >= 0.01)
        return;

    // Park Gear
    if (buttonState.at(joystick::ButtonPos::PARK) == 1
        && _prevButtonState.at(joystick::ButtonPos::PARK) == 0) {
        // if (out.gearPosition < maxGear)
            out.shift.data = eGearPosition::GEARPOSITION_PARK;
    }

    // REVERSE Gear
    if (buttonState.at(joystick::ButtonPos::REVERSE) == 1
        && _prevButtonState.at(joystick::ButtonPos::REVERSE) == 0) {
        // if (out.gearPosition > minGear)
            out.shift.data = eGearPosition::GEARPOSITION_REVERSE;
    }

    // NEUTRAL Gear
    if (buttonState.at(joystick::ButtonPos::NEUTRAL) == 1
        && _prevButtonState.at(joystick::ButtonPos::NEUTRAL) == 0) {
        // if (out.gearPosition > minGear)
            out.shift.data = eGearPosition::GEARPOSITION_NEUTRAL;
    }

    // DRIVE Gear
    if (buttonState.at(joystick::ButtonPos::DRIVE) == 1
        && _prevButtonState.at(joystick::ButtonPos::DRIVE) == 0) {
        // if (out.gearPosition > minGear)
            out.shift.data = eGearPosition::GEARPOSITION_DRIVE;
    }
    _prevButtonState.at(joystick::ButtonPos::PARK) = buttonState.at(joystick::ButtonPos::PARK);
    _prevButtonState.at(joystick::ButtonPos::REVERSE) = buttonState.at(joystick::ButtonPos::REVERSE);
    _prevButtonState.at(joystick::ButtonPos::NEUTRAL) = buttonState.at(joystick::ButtonPos::NEUTRAL);
    _prevButtonState.at(joystick::ButtonPos::DRIVE) = buttonState.at(joystick::ButtonPos::DRIVE);
}

void CommandCreator::set_indicator(tod_msgs::ControlCmd &out, const std::vector<int> &buttonState) {
    if (buttonState.at(joystick::ButtonPos::INDICATOR_LEFT) == 1
        && _prevButtonState.at(joystick::ButtonPos::INDICATOR_LEFT) == 0) {
        if(out.indicator.data == 1){
            out.indicator.data = 0;
        }
        else{
            if(out.indicator.data != 3)
                out.indicator.data = 1;
        }
         // Indicator Left

    } else if (buttonState.at(joystick::ButtonPos::INDICATOR_RIGHT) == 1
               && _prevButtonState.at(joystick::ButtonPos::INDICATOR_RIGHT) == 0) {
        if(out.indicator.data == 2){
            out.indicator.data = 0;
        }
        else{
            if(out.indicator.data != 3)
                out.indicator.data = 2;
        }
         // Indicator Right

    } else if(buttonState.at(joystick::ButtonPos::EMERGENCY_SIGNAL) == 1
                && _prevButtonState.at(joystick::ButtonPos::EMERGENCY_SIGNAL) == 0){
        if(out.indicator.data != 3){
            out.indicator.data = 3;
        }else{
            out.indicator.data = 0; 
        }// Emergency Indicator
        
    }
    _prevButtonState.at(joystick::ButtonPos::INDICATOR_LEFT) = buttonState.at(joystick::ButtonPos::INDICATOR_LEFT);
    _prevButtonState.at(joystick::ButtonPos::INDICATOR_RIGHT) = buttonState.at(joystick::ButtonPos::INDICATOR_RIGHT);
    _prevButtonState.at(joystick::ButtonPos::EMERGENCY_SIGNAL) = buttonState.at(joystick::ButtonPos::EMERGENCY_SIGNAL);
}

void CommandCreator::set_remote(tod_msgs::ControlCmd &out, const std::vector<int> &buttonState,
        const float &currentVelocity) {

    if (currentVelocity >= 0.01)
        return;

    // set remote signal
    if (buttonState.at(joystick::ButtonPos::REMOTE) == 1
        && _prevButtonState.at(joystick::ButtonPos::REMOTE) == 0) {
            if(out.remote_flag == 0) _remote_flag = 1;
            else if(out.remote_flag == 1) _remote_flag = 0;
            out.remote_flag = _remote_flag;
    }
    
    _prevButtonState.at(joystick::ButtonPos::REMOTE) = buttonState.at(joystick::ButtonPos::REMOTE);
}

void CommandCreator::set_video_stream(tod_msgs::ControlCmd &out, const std::vector<int> &buttonState,
        const float &currentVelocity) {

    if (currentVelocity >= 0.01)
        return;

    // video streaming signal
    if (buttonState.at(joystick::ButtonPos::VIDEO) == 1
        && _prevButtonState.at(joystick::ButtonPos::VIDEO) == 0) {
           if(out.stream_flag  == 0) _stream_flag = 1;
           else if(out.stream_flag  == 1) _stream_flag = 0;
            
            out.stream_flag = _stream_flag;
    }
    
    _prevButtonState.at(joystick::ButtonPos::VIDEO) = buttonState.at(joystick::ButtonPos::VIDEO);
}

// void CommandCreator::set_avm(tod_msgs::ControlCmd &out, const std::vector<int> &buttonState,
//         const float &currentVelocity) {

//     if (currentVelocity >= 0.01)
//         return;

//     // avm signal
//     if (buttonState.at(joystick::ButtonPos::AVM) == 1
//         && _prevButtonState.at(joystick::ButtonPos::AVM) == 0) {
//             if(out.avm_flag == 0) _avm_flag = 1;
//             else if(out.avm_flag == 1) _avm_flag = 0;
//             out.avm_flag = _avm_flag;
//     }
    
//     _prevButtonState.at(joystick::ButtonPos::AVM) = buttonState.at(joystick::ButtonPos::AVM);
// }

void CommandCreator::set_emergency_stop(tod_msgs::ControlCmd &out, const std::vector<int> &buttonState) {

    // if (currentVelocity <= 0.01)
    //     return;

    // AEB signal
    if (buttonState.at(joystick::ButtonPos::AEB_FLAG) == 1
        && _prevButtonState.at(joystick::ButtonPos::AEB_FLAG) == 0) {
            if(out.aeb_flag == 0) _aeb_flag = 1;
            else if(out.aeb_flag == 1) _aeb_flag = 0;
            out.aeb_flag = _aeb_flag;
    }
    
    _prevButtonState.at(joystick::ButtonPos::AEB_FLAG) = buttonState.at(joystick::ButtonPos::AEB_FLAG);
}


// void CommandCreator::set_parking_brake(tod_msgs::ControlCmd &out, const std::vector<int> &buttonState,
//         const float &currentVelocity) {

//     if (currentVelocity >= 0.01)
//         return;

//     // park signal
//     if (buttonState.at(joystick::ButtonPos::PARK_FLAG) == 1
//         && _prevButtonState.at(joystick::ButtonPos::PARK_FLAG) == 0) {
//             _parking_flag=!_parking_flag;
//             out.parking_flag = _parking_flag;
//     }
    
//     _prevButtonState.at(joystick::ButtonPos::PARK_FLAG) = buttonState.at(joystick::ButtonPos::PARK_FLAG);
// }

void CommandCreator::set_steer_mode(tod_msgs::ControlCmd &out, const std::vector<int> &buttonState,
        const float &currentVelocity) {

    if (currentVelocity >= 0.01)
        return;

    // steer mode
    if (buttonState.at(joystick::ButtonPos::STEER_MODE) == 1
        && _prevButtonState.at(joystick::ButtonPos::STEER_MODE) == 0) {
            _steering_mode++;
            if(_steering_mode>2){
                _steering_mode = 0;
            }
            out.steer_mode = _steering_mode;
    }
    _prevButtonState.at(joystick::ButtonPos::STEER_MODE) = buttonState.at(joystick::ButtonPos::STEER_MODE);

}

void CommandCreator::set_drive_mode(tod_msgs::ControlCmd &out, const std::vector<int> &buttonState,
        const float &currentVelocity) {

    if (currentVelocity >= 0.01)
        return;

    // drive signal
    if (buttonState.at(joystick::ButtonPos::DRIVE_MODE) == 1
        && _prevButtonState.at(joystick::ButtonPos::DRIVE_MODE) == 0) {

            _drive_mode++;
            if(_drive_mode>1){
                _drive_mode = 0;
            }
            out.drive_mode = _drive_mode;
    }
    
    _prevButtonState.at(joystick::ButtonPos::DRIVE_MODE) = buttonState.at(joystick::ButtonPos::DRIVE_MODE);
}

void CommandCreator::init_control_messages() {
    // _controlCmdMsg.operator_id = 0;
    // _controlCmdMsg.control_type = tod_msgs::Status::CONTROL_MODE_DIRECT;
    _controlCmdMsg.control.throttle = 0;
    _controlCmdMsg.control.brake = 0;
    _controlCmdMsg.control.steering_angle = 0;
    _controlCmdMsg.indicator.data = 0;
    _controlCmdMsg.shift.data = 0;

    _controlCmdMsg.remote_flag = 0;
    _controlCmdMsg.stream_flag = 0;
    _controlCmdMsg.aeb_flag = 0;
    // _controlCmdMsg.parking_flag = false;

    _controlCmdMsg.steer_mode = 0;
    _controlCmdMsg.drive_mode = 0;

}

double CommandCreator::map(double output, double in_min, double in_max, double out_min, double out_max)
{
    return (output - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}