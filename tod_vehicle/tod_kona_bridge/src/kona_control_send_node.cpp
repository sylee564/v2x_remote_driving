#include <ros/ros.h>
#include <math.h>
#include <iostream>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <tod_helper/vehicle/Model.h>
#include "tod_msgs/VehicleEnums.h"
#include "tod_msgs/ControlCmd.h"
#include "tod_msgs/VehicleData.h"
// #include "tod_msgs/PrimaryControlCmd.h"
// #include "tod_msgs/SecondaryControlCmd.h"

#define PI 3.141592

double tau = 0;
double ts = 0;
double pre_y = 0;
double error = 0;
double control=0;
float v_previous=0.0;
float _maxSpeedms;

typedef struct DirectControlOutput{
    //Primary Cmd
    int steeringWheelAngle;
    int target_speed;
    float target_acceleration;

    //Secondary Cmd
    bool remote_flag;
    uint8_t indicator;
    uint8_t gear;
    // uint8_t honk;
    // uint8_t wiper;
    // uint8_t headLight;
    // uint8_t flashLight;
    
    //vechicle data
    float vehicle_velocity;
    uint8_t vehicle_gear;
    int64_t stamp;

    DirectControlOutput() :
        steeringWheelAngle(0), target_speed(0), target_acceleration(0), gear(0), 
        indicator(0), remote_flag(false), vehicle_velocity(0.0),  vehicle_gear(0){}
} tDirectControlOutput;

tDirectControlOutput controlCommand;
int s; 
struct sockaddr_can addr;
struct ifreq ifr;
struct can_frame frame;
unsigned char ALCNT{0};

void init_mode(){
    controlCommand.target_acceleration = 0;
    controlCommand.target_speed = 0;
    controlCommand.gear=0;
    controlCommand.indicator=0;
}

// operator control command callback function
void callback_control_command(const tod_msgs::ControlCmdConstPtr& msg) {
    controlCommand.steeringWheelAngle = static_cast<int>(tod_helper::Vehicle::Model::rad2deg(msg->steeringWheelAngle));
    controlCommand.target_speed = static_cast<int>(msg->velocity);
    controlCommand.target_acceleration = floor(msg->acceleration*100)/100;
    controlCommand.remote_flag=msg->remote_flag;
    controlCommand.gear = msg->gearPosition;
    controlCommand.indicator = msg->indicator;
    // controlCommand.honk=msg->honk;
    // controlCommand.wiper=msg->wiper;
    // controlCommand.headLight=msg->headLight;
    // controlCommand.flashLight=msg->flashLight;
    switch (msg->gearPosition) {
        case 0: controlCommand.gear = GEARPOSITION_PARK; break;
        case 1: controlCommand.gear = GEARPOSITION_REVERSE; break;
        case 2: controlCommand.gear = GEARPOSITION_NEUTRAL; break;
        case 3: controlCommand.gear = GEARPOSITION_DRIVE; break;
    }
}

// vehicle status callback function
void callback_vehicle_status(const tod_msgs::VehicleData::ConstPtr &msg)
{
    controlCommand.vehicle_velocity = msg->vehicle_velocity;
    controlCommand.vehicle_gear = msg->gearPosition;
    // controlCommand.ACC_En_Status = KONA->ACC_En_Status;
    // controlCommand.EPS_En_Status = KONA->EPS_En_Status;
}

double LPF(double input)
{
    tau = 1 / (200 * 2 * PI);
    ts = 0.0001;
    pre_y = ((tau * pre_y) + (ts * input)) / (tau + ts);
    return pre_y;
}

double PID(float target_, float current_)
{
    float kp=1;
    float ki=1;
    float kd=0.01;
    float v_previous=0.0;
    float t_last=0.0;
    float u, derivate, e_previous;
    static ros::Time prevTime = 0;
    static ros::Time current_time = ros::Time::now();
    float current_e = target_ - current_;
    ros::Duration dt = ros::Time::now() - prevTime;
    
    float proportional = kp*current_e;
    float variable_e = variable_e+current_e*dt.toSec();
    float integral=ki*variable_e;

    if(dt.toSec()==0){
        derivate=0;
    }
    else{
        derivate=kd*((error-e_previous)/dt.toSec());
    }
    
    u = proportional + integral + derivate;

    prevTime = current_time;
    return u;
}


void secondary_control(tDirectControlOutput &controlCommand)
{
    frame.can_id = 0x156; // (0x156) Mo_Conf
    frame.can_dlc = 8;

    frame.data[0] = controlCommand.remote_flag;         // APM_IGNORE, (APM_En / APM_D_En)
    frame.data[1] = 200 * 1.0; // APM_Slevel(Steering angular speed level) 0.5*(real_value), [0, 510]
    frame.data[2] = controlCommand.remote_flag;      // ASM_StopRequest(Req AEB action : 0x12), ASM_En(En : 0x02, D_En : 0x00)
    frame.data[3] = 0;
    frame.data[4] = 0;
    frame.data[5] = ((controlCommand.gear<< 4)& 0b11110000) | (controlCommand.indicator & 0b00000111);
    frame.data[6] = 0;
    frame.data[7] = ALCNT;
    ALCNT += 1;    
}

void main_control(int steeringWheel, double acc)
{
     if (acc > 1.0)
        acc = 1.0;
    if (acc < -3)
        acc = -3;
       
    frame.can_id = 0x157; // (0x157) Mo_Val
    frame.can_dlc = 5;

    frame.data[0] = (u_int8_t)(((10 * steeringWheel) & 0x00ff));
    frame.data[1] = (u_int8_t)((((10 * steeringWheel) & 0xff00) >> 8));
    frame.data[2] = 0; // set display
    frame.data[3] = (u_int8_t)((u_int16_t)(100 * (acc+ 10.23)) & 0x00ff);
    frame.data[4] = (u_int8_t)(((u_int16_t)(100 * (acc+ 10.23)) >> 8) & 0x00ff);
}

//main controller
void primary_control(tDirectControlOutput &controlCommand)
{
    controlCommand.steeringWheelAngle =LPF(controlCommand.steeringWheelAngle); // LPF steer input
    
    if (controlCommand.vehicle_gear == 0 || controlCommand.vehicle_gear == 2)// 250ms
    {
        main_control(controlCommand.steeringWheelAngle,  0.00);
    }
    else
    {
        // controlCommand.accelerator_ = PID(controlCommand.target_kmh_, controlCommand.read_kmh_); // raw pid input
        if(controlCommand.vehicle_velocity == 0 && controlCommand.target_acceleration<0){
            controlCommand.target_acceleration = 0.00;
        }
        // else if(controlCommand.vehicle_velocity>_maxSpeedms){
        //     controlCommand.accelerator_-=0.10;
        // }
        main_control(controlCommand.steeringWheelAngle, controlCommand.target_acceleration);
    }
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "CammandToVehicle");
	ros::NodeHandle n;
	ros::Subscriber subControlCommand = n.subscribe("/Vehicle/CommandCreation/control_cmd_data",
            1, callback_control_command);
    ros::Subscriber subVehicleStatus = n.subscribe("/Vehicle/VehicleBridge/vehicle_data",
            1, callback_vehicle_status);
	ros::Rate loopRate(100);
    ros::Duration(1).sleep();
	float _maxSpeedms;

    n.param<float>("/Vehicle/Control/command_control/maxVelocity", _maxSpeedms, 0);
	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Socket");
		return 1;
	}

    const char* can_dev="can0";
	strcpy(ifr.ifr_name, can_dev );
	ioctl(s, SIOCGIFINDEX, &ifr);

	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

    fcntl(s, F_SETFL, O_NONBLOCK);
	if (bind(s, reinterpret_cast<struct sockaddr *> (&addr), sizeof(addr)) < 0) {
		perror("Bind");
		return 1;
	}
	
	while(ros::ok()){
		ros::spinOnce();

        secondary_control(controlCommand);

		if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
			perror("Write");
			return 1;
		}

        primary_control(controlCommand);

		if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
			perror("Write");
			return 1;
		}
		loopRate.sleep();
		
	}
	
	if (close(s) < 0) {
		perror("Close");
		return 1;
    }

	return 0;
}