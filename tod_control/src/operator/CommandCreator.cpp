#include "CommandCreator.h"


CommandCreator::CommandCreator(ros::NodeHandle& nodeHandle) :_nh(nodeHandle)
{
    /*ROS node create*/
    _joystickSubs = nodeHandle.subscribe("/Operator/InputDevices/joystick", 1,
                                         &CommandCreator::callback_joystick_msg, this);
    _statusSubs = nodeHandle.subscribe("/Operator/Manager/status_msg", 1,
                                       &CommandCreator::callback_status_msg, this);
    _vehicleSubs = nodeHandle.subscribe("/Operator/VehicleBridge/vehicle_data", 1,
                                       &CommandCreator::callback_vehicle_data_msg, this);
    _controlCmdPub = nodeHandle.advertise<tod_msgs::ControlCmd>("control_cmd_data", 1);

    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::INDICATOR_LEFT, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::INDICATOR_RIGHT, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::EMERGENCY_LIGHT, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::INCREASE_SPEED, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::DECREASE_SPEED, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::PARK, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::REVERSE, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::NEUTRAL, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::DRIVE, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::REMOTE, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::VIDEO, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::AVM, 0));

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
                                                   
    ros::Duration(1).sleep();
    ROS_WARN("Please Wait.................................");
}
void CommandCreator::run() {
    ros::Rate loop_rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        //  std::cout<<"_vehParams check!!!"<<std::endl;
        //  bool test1=_vehParams->vehicle_id_has_changed();
        //  printf("%d", test1);
        // if (_vehParams->vehicle_id_has_changed()) {
        //     _vehParams->load_parameters();
        // }
        if (_joystickInputSet && _status == tod_msgs::Status::TOD_STATUS_TELEOPERATION) {
            _controlCmdMsg.header.stamp = ros::Time::now();
            
            _controlCmdPub.publish(_controlCmdMsg);
        }
        _joystickInputSet = false;
        loop_rate.sleep();
    }
}

void CommandCreator::callback_joystick_msg(const sensor_msgs::Joy::ConstPtr& msg) {
    if (_status == tod_msgs::Status::TOD_STATUS_TELEOPERATION) {
        set_remote(_controlCmdMsg, msg->buttons, _controlCmdMsg.velocity);
        set_video_stream(_controlCmdMsg, msg->buttons, _controlCmdMsg.velocity);
        _controlCmdMsg.operator_id = Oper_ID;
        _controlCmdMsg.control_type = _control_type;
        if(_remote_flag){
            calculate_steering_wheel_angle(_controlCmdMsg, msg->axes);
            calculate_desired_velocity(_controlCmdMsg, msg, _controlCmdMsg.gearPosition);
            set_gear(_controlCmdMsg, msg->buttons, _controlCmdMsg.velocity);
            set_indicator(_controlCmdMsg, msg->buttons);
            // set_light(_secondaryControlMsg, msg->buttons);
        }
        _joystickInputSet = true;
    }
}

void CommandCreator::callback_vehicle_data_msg(const tod_msgs::VehicleData::ConstPtr &msg) {
    vehicle_velocity_ = msg->vehicle_velocity;
}

void CommandCreator::calculate_steering_wheel_angle(tod_msgs::ControlCmd& out,
        const std::vector<float>& axes) {
    static ros::Time tPrev;
    static ros::Duration dur;
    static double oldSetSWA{0.0};
    // calc desired SWA
    double newDesiredSWA = axes.at(joystick::AxesPos::STEERING);
    // double newDesiredSWA = axes.at(joystick::AxesPos::STEERING) * _vehParams->get_max_swa_rad();
     if (_constraintSteeringRate) { // constraint steering rate
        dur = ros::Time::now() - tPrev;
        double newSetSWA = std::min(newDesiredSWA, oldSetSWA + dur.toSec() * _maxSteeringWheelAngleRate);
        newSetSWA = std::max(newSetSWA, oldSetSWA - dur.toSec() * _maxSteeringWheelAngleRate);
        oldSetSWA = newSetSWA;
        out.steeringWheelAngle = newSetSWA;
        tPrev = ros::Time::now();
    } else { // output unconstraint SWA
        out.steeringWheelAngle = newDesiredSWA;
    }

    out.steeringWheelAngle =  -newDesiredSWA*7.8538;  //maximum steering wheel angle radian 
    if(_controlCmdMsg.indicator==2 ){
        if(out.steeringWheelAngle* 180.0 / 3.1415 > 70){
            _controlCmdMsg.indicator=0;
        }
    }
    else if(_controlCmdMsg.indicator==4){
        if(out.steeringWheelAngle* 180.0 / 3.1415 < -70){
            _controlCmdMsg.indicator=0;
        }
    }
    // if (_invertSteeringInGearReverse &&
    //     _secondaryControlMsg.gearPosition == eGearPosition::GEARPOSITION_REVERSE)
    //     out.steeringWheelAngle = -out.steeringWheelAngle;

}

void CommandCreator::calculate_desired_velocity(tod_msgs::ControlCmd &out,
        const sensor_msgs::Joy::ConstPtr &msg, const int gear)
{   
    if (gear == eGearPosition::GEARPOSITION_PARK || gear == eGearPosition::GEARPOSITION_NEUTRAL) {
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

    // changeOperator = (msg->axes.at(joystick::AxesPos::THROTTLE) - msg->axes.at(joystick::AxesPos::BRAKE)) / 2.0;
    
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

        // Saturate Speed integration and limit Acceleration
        if (out.velocity > _maxSpeedms) {
            out.velocity = _maxSpeedms; //Limit demanded speed
        } else if (out.velocity < 0) {
            out.velocity = 0; //Limit demanded speed to zero
        }

    }
    else{
        static float deadzoneThrottle{0.05}, deadzoneBrake{0.05};
        if(msg->axes.at(joystick::AxesPos::THROTTLE)>0 || msg->axes.at(joystick::AxesPos::BRAKE)>0 ){
            // static ros::Time prevTime = ros::Time::now();
            changeOperator = (msg->axes.at(joystick::AxesPos::THROTTLE) - msg->axes.at(joystick::AxesPos::BRAKE)) / 2.0;
            
            if (changeOperator >= 0) {
                a_soll = _maxAcceleration * (std::max(changeOperator, (float) deadzoneThrottle) - deadzoneThrottle); //acc
            } else {
                a_soll = _maxDeceleration * (std::min(changeOperator, (float) -deadzoneBrake) + deadzoneBrake); // decelerate
            }

            ros::Duration dt = ros::Time::now() - prevTime;
            prevTime = ros::Time::now();
            out.velocity = out.velocity+ dt.toSec() * a_soll;

            if (vehicle_velocity_> _maxSpeedms) {
                vehicle_velocity_ = _maxSpeedms; //Limit demanded speed
                if(a_soll>=0){
                    a_soll=-0.10;
                }
            } else if (out.velocity< 0) {
                out.velocity = 0; //Limit demanded speed to zero
            }
        }
        else if(msg->axes.at(joystick::AxesPos::THROTTLE)==-1.0 && msg->axes.at(joystick::AxesPos::BRAKE)==-1.0) {
        
            changeOperator=((msg->axes.at(joystick::AxesPos::THROTTLE)/2.0)-msg->axes.at(joystick::AxesPos::BRAKE));
            if (vehicle_velocity_< 2) {
                if (changeOperator >= 0) {
                    a_soll = _maxAcceleration*0.3 * (std::max(changeOperator, (float) deadzoneThrottle) - deadzoneThrottle); //acc
                } else {
                    a_soll = _maxDeceleration * (std::min(changeOperator, (float) -deadzoneBrake) + deadzoneBrake); // decelerate
                }
            }
            else if(vehicle_velocity_>2){
                a_soll=(msg->axes.at(joystick::AxesPos::THROTTLE)/2.0)*0.2;
            }

            ros::Duration dt = ros::Time::now() - prevTime;
            prevTime = ros::Time::now();
            out.velocity = out.velocity+ dt.toSec() * a_soll;
            
        }
    }
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
        

    out.acceleration = a_soll;
}

void CommandCreator::callback_status_msg(const tod_msgs::Status &msg) {
    if (_status == tod_msgs::Status::TOD_STATUS_TELEOPERATION
        && msg.tod_status != tod_msgs::Status::TOD_STATUS_TELEOPERATION) {
        init_control_messages();
        // printf("initialization");
    }

    _control_type=msg.operator_control_mode;
    _status = msg.tod_status;
    
}

void CommandCreator::set_gear(tod_msgs::ControlCmd &out, const std::vector<int> &buttonState,
        const float &currentVelocity) {
    // static int maxGear{5};
    // static int minGear{0};

    if (currentVelocity >= 0.01)
        return;

    // Park Gear
    if (buttonState.at(joystick::ButtonPos::PARK) == 1
        && _prevButtonState.at(joystick::ButtonPos::PARK) == 0) {
        // if (out.gearPosition < maxGear)
            out.gearPosition = 0;
    }

    // REVERSE Gear
    if (buttonState.at(joystick::ButtonPos::REVERSE) == 1
        && _prevButtonState.at(joystick::ButtonPos::REVERSE) == 0) {
        // if (out.gearPosition > minGear)
            out.gearPosition = 1;
    }

    // NEUTRAL Gear
    if (buttonState.at(joystick::ButtonPos::NEUTRAL) == 1
        && _prevButtonState.at(joystick::ButtonPos::NEUTRAL) == 0) {
        // if (out.gearPosition > minGear)
            out.gearPosition = 2;
    }

    // DRIVE Gear
    if (buttonState.at(joystick::ButtonPos::DRIVE) == 1
        && _prevButtonState.at(joystick::ButtonPos::DRIVE) == 0) {
        // if (out.gearPosition > minGear)
            out.gearPosition = 3;
    }
    _prevButtonState.at(joystick::ButtonPos::PARK) = buttonState.at(joystick::ButtonPos::PARK);
    _prevButtonState.at(joystick::ButtonPos::REVERSE) = buttonState.at(joystick::ButtonPos::REVERSE);
    _prevButtonState.at(joystick::ButtonPos::NEUTRAL) = buttonState.at(joystick::ButtonPos::NEUTRAL);
    _prevButtonState.at(joystick::ButtonPos::DRIVE) = buttonState.at(joystick::ButtonPos::DRIVE);
}

void CommandCreator::set_indicator(tod_msgs::ControlCmd &out, const std::vector<int> &buttonState) {
    if (buttonState.at(joystick::ButtonPos::INDICATOR_LEFT) == 1
        && _prevButtonState.at(joystick::ButtonPos::INDICATOR_LEFT) == 0) {
        if(out.indicator==2){
            out.indicator = 0;
        }
        else{
            if(out.indicator!=1)
                out.indicator = 2;
        }
         // Indicator Left

    } else if (buttonState.at(joystick::ButtonPos::INDICATOR_RIGHT) == 1
               && _prevButtonState.at(joystick::ButtonPos::INDICATOR_RIGHT) == 0) {
        if(out.indicator==4){
            out.indicator = 0;
        }
        else{
            if(out.indicator!=1)
                out.indicator = 4;
        }
         // Indicator Right

    } else if(buttonState.at(joystick::ButtonPos::EMERGENCY_LIGHT) == 1
                && _prevButtonState.at(joystick::ButtonPos::EMERGENCY_LIGHT) == 0){
        if(out.indicator!=1){
            out.indicator=1;
        }else{
            out.indicator = 0; 
        }// Emergency Indicator
        
    }
    _prevButtonState.at(joystick::ButtonPos::INDICATOR_LEFT) = buttonState.at(joystick::ButtonPos::INDICATOR_LEFT);
    _prevButtonState.at(joystick::ButtonPos::INDICATOR_RIGHT) = buttonState.at(joystick::ButtonPos::INDICATOR_RIGHT);
    _prevButtonState.at(joystick::ButtonPos::EMERGENCY_LIGHT) = buttonState.at(joystick::ButtonPos::EMERGENCY_LIGHT);
}

void CommandCreator::set_remote(tod_msgs::ControlCmd &out, const std::vector<int> &buttonState,
        const float &currentVelocity) {
    // static int maxGear{5};
    // static int minGear{0};

    if (currentVelocity >= 0.01)
        return;

    // Park Gear
    if (buttonState.at(joystick::ButtonPos::REMOTE) == 1
        && _prevButtonState.at(joystick::ButtonPos::REMOTE) == 0) {
            _remote_flag=!_remote_flag;
            out.remote_flag = _remote_flag;
    }
    
    _prevButtonState.at(joystick::ButtonPos::REMOTE) = buttonState.at(joystick::ButtonPos::REMOTE);
}

void CommandCreator::set_video_stream(tod_msgs::ControlCmd &out, const std::vector<int> &buttonState,
        const float &currentVelocity) {
    // static int maxGear{5};
    // static int minGear{0};

    if (currentVelocity >= 0.01)
        return;

    // video streaming signal
    if (buttonState.at(joystick::ButtonPos::VIDEO) == 1
        && _prevButtonState.at(joystick::ButtonPos::VIDEO) == 0) {
            _stream_flag=!_stream_flag;
            out.streaming_flag = _stream_flag;
    }

    // video streaming signal
    if (buttonState.at(joystick::ButtonPos::AVM) == 1
        && _prevButtonState.at(joystick::ButtonPos::AVM) == 0) {
            _avm_flag=!_avm_flag;
            out.avm_flag = _avm_flag;
    }
    
    _prevButtonState.at(joystick::ButtonPos::VIDEO) = buttonState.at(joystick::ButtonPos::VIDEO);
    _prevButtonState.at(joystick::ButtonPos::AVM) = buttonState.at(joystick::ButtonPos::AVM);
}

void CommandCreator::init_control_messages() {
    _controlCmdMsg.operator_id = 0;
    _controlCmdMsg.control_type = tod_msgs::Status::CONTROL_MODE_DIRECT;
    _controlCmdMsg.remote_flag=false;
    _controlCmdMsg.streaming_flag=false;
    _controlCmdMsg.acceleration = 0;
    _controlCmdMsg.steeringWheelAngle = 0;
    _controlCmdMsg.velocity = 0;
    _controlCmdMsg.gearPosition = 0;
    _controlCmdMsg.remote_flag = false;
    _controlCmdMsg.streaming_flag = false;
    _controlCmdMsg.avm_flag = false;
    // _controlCmdMsg.wiper = 0;
    // _controlCmdMsg.honk=0;
    // _controlCmdMsg.headLight=0;
    // _controlCmdMsg.flashLight=0;
}

// void CommandCreator::timerCallback(const ros::TimerEvent &)
// {
    
//     // if(1){
//     if(vehicle_status==1 || vehicle_status==2){
//     // if((EPS_En_Status==1) && (ACC_En_Status==1)){
//         _joystick.updateFfDevice();
//         set_flag(OperMsg_);
//         OperMsg_.Indicator=_joystick.indicator;
//         OperMsg_.gear=_joystick.oper_gears;
//         oper_gears=_joystick.oper_gears;
//         m_current_angle=_joystick.m_current_angle;
//         m_current_accelerator=_joystick.m_current_accelerator;
//         m_current_brake=_joystick.m_current_brake;
//         _joystick.velocity=vehicle_velocity_;
//         set_gear(OperMsg_);
//         calculate_desired_velocity(OperMsg_);

//         static ros::Time resetTime{ros::Time::now()};
//         if (ros::Time::now() >= resetTime + ros::Duration(25.0)) {
//             if (_joystick.ok()) _joystick.reset();
//             resetTime = ros::Time::now();
//         }
//     }
//     else if(vehicle_status==0){
//         OperMsg_.Indicator=0;
//         _joystick.indicator=0;
//         OperMsg_.gear=0;
//         _joystick.oper_gears=0;
//         OperMsg_.status="Parking";
//         // OperMsg_.remote_flag=false;
//         // _joystick.remote_flag=false;
//     }
//     OperMsg_.gear_cur=vehicle_gear;
//     OperMsg_.header.stamp=ros::Time::now();
//     OperMsg_.header.frame_id="Operator_command";
//     G29_pub_.publish(OperMsg_);
// }

// void CommandCreator::KonaCb(const tod_msgs::kona_status::ConstPtr &KONA)
// {
//     vehicle_steer_ = KONA->steeringWheelAngle;         //[-500, 500]
//     vehicle_steer_Tq_ = KONA->steer_Tq;         //[-20.48, 20.47]
//     vehicle_velocity_ = KONA->vehicle_velocity;           //[0, 255]
//     vehicle_Long_acc_ = KONA->LONG_ACCEL;       // [-10.23, 10.23]
//     vehicle_Lat_acc_ = KONA->LAT_ACCEL;         //[-10.23, 10.23]
//     vehicle_Yaw_rate_ = KONA->YAW_RATE;         //[-40.95, 40.94]
//     vehicle_Wheel_spd_fl_ = KONA->WHEEL_SPD_FL; //[0, 511.96875]
//     vehicle_Wheel_spd_fr_ = KONA->WHEEL_SPD_FR; //[0, 511.96875]
//     vehicle_Wheel_spd_rl_ = KONA->WHEEL_SPD_RL; //[0, 511.96875]
//     vehicle_WHeel_spd_rr_ = KONA->WHEEL_SPD_RR; //[0, 511.96875]
//     vehicle_gear_status=KONA->Gear_Control_Status;
//     vehicle_gear=KONA->Gear_Cur;
//     // EPS_Control_Status=KONA->EPS_Control_Status;
//     // ACC_Control_Status=KONA->ACC_Control_Status;
//     // EPS_En_Status=KONA->EPS_En_Status;
//     // ACC_En_Status=KONA->ACC_En_Status;
//     vehicle_status=KONA->vehicle_status;
//     if(vehicle_status==1 || vehicle_status==2){
//         remote_status=true;
//         _joystick.remote_status=remote_status;
//     }
//     else{
//         remote_status=false;
//         _joystick.remote_status=remote_status;
//     }
// }


// void CommandCreator::set_gear(tod_msgs::oper& out)
// {
//     if(vehicle_gear==0){
//         out.status="Parking";
//     }
//     else if(vehicle_gear==7){
//         out.status="Reverse";
//     }
//     else if(vehicle_gear==6){
//         out.status="Neutral";
//     }
//     else if(vehicle_gear==5){
//         out.status="Drive";
//     }
// }

// void CommandCreator::set_flag(tod_msgs::oper& out)
// {
//     out.remote_flag=_joystick.remote_flag;
//     out.stream_flag=_joystick.stream_flag;
//     out.avm_flag=_joystick.avm_flag;
//     out.recording_flag=_joystick.recording_flag;
//     out.parking_seg_flag=_joystick.parking_seg_flag;
// }

// void CommandCreator::callback_force_feedback(const std_msgs::Float64ConstPtr &msg) {
//         if (_joystick.ok()) {
//             _joystick.set_force_feedback(msg->data);
//         } else {
//             ROS_WARN_ONCE("%s: joystick not ok - not setting force feedback",
//                           ros::this_node::getName().c_str());
//         }
// }

// void CommandCreator::SteerPID(double target_, double current_)
// {
//     // ROS_WARN("target : %f\t current : %f", target_, current_);
//     targetDeg = target_;
//     currentDeg = current_;
//     error = targetDeg - currentDeg;

//     if (abs(error) < St_offset)
//     {
//         control = 0;
//     }
//     else
//     {
//         control = (error * Kp);

//         control = (control > 0.0) ? std::min(control, 0.6) : std::max(control, -0.6);
//     }
//     // ROS_WARN("Control value : %f, %d", control, vehicle_gear_status);
//     SteerCTRL(control);
// }
