#include <ros/ros.h>
#include <math.h>
#include <iostream>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include "tod_msgs/VehicleData.h"
#include "tod_msgs/VehicleEnums.h"
#include "tod_msgs/SafetyDriverStatus.h"
// #include "tod_msgs/PVD_data.h"
#include <tod_helper/vehicle/Model.h>
#include "geometry_msgs/TwistWithCovarianceStamped.h"

struct SafetyDriverStatus {
    bool vehicle_emergency_stop_pressed;
    bool vehicle_EPS_approved;
    bool vehicle_ACC_approved;
    bool vehicle_EPS_Status;
    bool vehicle_ACC_Status;
};

typedef struct VehicleData {
    
    uint8_t EPS_En_Status;
    uint8_t EPS_Control_Board_Status;
    uint8_t EPS_Control_Status;
    uint8_t ACC_En_Status;
    uint8_t ACC_Control_Board_Status;
    uint8_t ACC_Control_Status;

    float read_steer_;
    float read_steer_Tq_;

    float read_kmh_;
    float read_long_accel_;
    float read_lat_accel_;

    float read_yaw_rate_;

    float read_BRK_CYLINDER_;

    float read_wheel_spd_fl;
    float read_wheel_spd_fr;
    float read_wheel_spd_rl;
    float read_wheel_spd_rr;

    uint8_t gear_cur;
    uint8_t gear_cmd;
    uint8_t gear_control_status;


    VehicleData() :
        EPS_En_Status(0), EPS_Control_Board_Status(0), EPS_Control_Status(0),ACC_En_Status(0),
        ACC_Control_Board_Status(0), ACC_Control_Status(0), read_steer_(0), read_steer_Tq_(0),
        read_kmh_(0), read_long_accel_(0), read_lat_accel_(0), read_yaw_rate_(0), read_BRK_CYLINDER_(0),
        read_wheel_spd_fl(0), read_wheel_spd_fr(0), read_wheel_spd_rl(0), read_wheel_spd_rr(0),
        gear_cur(0), gear_cmd(0), gear_control_status(0){}
} tVehicleData;

void vehicle_data_to_wheel_odometry_msg(const tVehicleData &vehicleData, geometry_msgs::TwistWithCovarianceStamped &msg) {
    static int ctrSend{0};
    msg.header.seq = ++ctrSend;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "rear_wheel_centered_frame";
    msg.twist.twist.linear.x = 1.03*(vehicleData.read_wheel_spd_rl+vehicleData.read_wheel_spd_rr)/2.0;
    msg.twist.twist.linear.y = 0;
    msg.twist.covariance = {0.00012,    0.0,    0.0,    0.0,    0.0,    0.0,
                            0.0,    0.00000001,    0.0,    0.0,    0.0,    0.0,
                            0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                            0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                            0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                            0.0,    0.0,    0.0,    0.0,    0.0,    0.0012};
}

void status_to_ros_msg(const SafetyDriverStatus& safety_driver_status, tod_msgs::SafetyDriverStatus &msg) {
    msg.vehicle_emergency_stop_released = !safety_driver_status.vehicle_emergency_stop_pressed;
    msg.vehicle_EPS_approved = safety_driver_status.vehicle_EPS_approved;
    msg.vehicle_ACC_approved = safety_driver_status.vehicle_ACC_approved;
    msg.vehicle_EPS_Status = safety_driver_status.vehicle_EPS_Status;
    msg.vehicle_ACC_Status = safety_driver_status.vehicle_ACC_Status;
}

// void vehicle_data_to_pvd_msg(const tVehicleData &vehicleData, tod_msgs::PVD_data &msg) {

//     msg.steering_wheel=vehicleData.read_steer_;
//     msg.acceleration=vehicleData.read_long_accel_;
//     msg.velocity =(uint8_t)vehicleData.read_kmh_;

//     switch (vehicleData.gear_cur) {
//         case GEARPOSITION_PARK: msg.gear_status = 0; break;
//         case GEARPOSITION_REVERSE: msg.gear_status = 1; break;
//         case GEARPOSITION_NEUTRAL: msg.gear_status = 2; break;
//         case GEARPOSITION_DRIVE: msg.gear_status = 3; break;
//     }

//     msg.latitude = 5.1234567;
//     msg.longitude = 1.2345678;
//     msg.heading = 45.123546;

//     msg.lat_approved = vehicleData.EPS_Control_Status;
//     msg.long_approved = vehicleData.ACC_Control_Status;

//     if(vehicleData.ACC_Control_Status==2 && vehicleData.EPS_Control_Status==2){
//         if(vehicleData.ACC_En_Status==1 && vehicleData.EPS_En_Status==1){
//             msg.vehicle_mode=1;
//         }
//     }
//     else{
//         msg.vehicle_mode=0;
//     }
// }

void vehicle_data_to_ros_msg(const tVehicleData &vehicleData, tod_msgs::VehicleData &msg) {

    static int ctrSend{0};

    msg.header.seq = ++ctrSend;
    msg.header.stamp = ros::Time::now();

    msg.steeringWheelAngle = tod_helper::Vehicle::Model::deg2rad(vehicleData.read_steer_);
    // msg.steeringWheelAngle = vehicleData.read_steer_;
    msg.steerWheelToque= vehicleData.read_steer_Tq_;

    msg.vehicle_velocity =tod_helper::Vehicle::Model::kph2mps(vehicleData.read_kmh_);
    // msg.longitudinalSpeed =vehicleData.read_kmh_;
    // msg.longitudinalAcc = vehicleData.read_long_accel_;

    msg.linearAcceleration.x = vehicleData.read_lat_accel_;
    msg.linearAcceleration.y = vehicleData.read_long_accel_;
    msg.linearAcceleration.z = 0.0;
    
    msg.curvature=vehicleData.read_yaw_rate_;
    msg.BRK_CYLINDER=vehicleData.read_BRK_CYLINDER_;

    msg.WHEEL_SPD_FL = vehicleData.read_wheel_spd_fl;
    msg.WHEEL_SPD_FR = vehicleData.read_wheel_spd_fr;
    msg.WHEEL_SPD_RL = vehicleData.read_wheel_spd_rl;
    msg.WHEEL_SPD_RR = vehicleData.read_wheel_spd_rr;

    switch (vehicleData.gear_cur) {
        case GEARPOSITION_PARK: msg.gearPosition = 0; break;
        case GEARPOSITION_REVERSE: msg.gearPosition = 1; break;
        case GEARPOSITION_NEUTRAL: msg.gearPosition = 2; break;
        case GEARPOSITION_DRIVE: msg.gearPosition = 3; break;
    }
    msg.gearPosition=vehicleData.gear_cur;
    msg.Gear_Cmd=vehicleData.gear_cmd;
    msg.Gear_Control_Status=vehicleData.gear_control_status;
}

int byte_handling(unsigned char low_, unsigned char high_)
{
    int out = 0;

    out = low_ | (high_ << 8);

    if (out < 32768)
        out = out;
    else if (out > 32768)
        out -= 0xffff;

    return out;
}

void inform_the_user_about_success_once() {
    static bool once{false};
    if (!once) {
        once = true;
        ROS_INFO("%s: Received first kona data from can_raw!",
            ros::this_node::getName().c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "VehicleDataToMsg");
	ros::NodeHandle n;
	ros::Publisher pubVehicleData=n.advertise<tod_msgs::VehicleData>("vehicle_data", 5);
    ros::Publisher pubSafetyDriverStatus = n.advertise<tod_msgs::SafetyDriverStatus>("safety_driver_status", 1);
    // ros::Publisher pubPVD_=n.advertise<tod_msgs::PVD_data>("PVD_data", 1);
    ros::Duration(1).sleep();
    
    tod_msgs::VehicleData vehicle_data_msg;
    tod_msgs::SafetyDriverStatus safety_driver_status_msg;
    // tod_msgs::PVD_data pvd_data_msg;

    tVehicleData vehicleData;
    SafetyDriverStatus safety_driver_status;

	int s, i; 
	int nbytes;
	struct sockaddr_can addr;
	struct ifreq ifr;
	struct can_frame frame;

	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Socket");
		return 1;
	}

	strcpy(ifr.ifr_name, "can0" );
	ioctl(s, SIOCGIFINDEX, &ifr);

	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("Bind");
		return 1;
	}

	struct can_filter rfilter[2];

	rfilter[0].can_id   = 0x710;
	rfilter[0].can_mask = 0xFF0;
	rfilter[1].can_id   = 0x720;
	rfilter[1].can_mask = 0xFF0;

	setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

	while(ros::ok()){

        ros::spinOnce();
		nbytes = read(s, &frame, sizeof(struct can_frame));
		if (nbytes < 0) {
			perror("Read");
			return 1;
		}
        
		if(frame.can_id==0x710){
			vehicleData.EPS_En_Status=frame.data[0]&0b00000001;
            vehicleData.EPS_Control_Board_Status=(frame.data[0]&0b00001110)>>1;
            vehicleData.EPS_Control_Status=frame.data[1]&0b00001111;
            vehicleData.read_steer_ = byte_handling(frame.data[2], frame.data[3]) * 0.1;             //[-500, 500]
            vehicleData.read_steer_Tq_ = (byte_handling(frame.data[5], frame.data[6]) - 2048) * 0.1; //[-20.48, 20.47]
		}
			
		if(frame.can_id==0x711){
			vehicleData.ACC_En_Status=frame.data[0]&0b00000001;
            vehicleData.ACC_Control_Board_Status=(frame.data[0]&0b00001110)>>1;
            vehicleData.ACC_Control_Status=frame.data[1]&0b00001111;
            vehicleData.read_kmh_ = frame.data[2];                                                     //[0, 255]
            vehicleData.read_long_accel_ =  (byte_handling(frame.data[4], frame.data[5]) - 1023) * 0.01; // [-10.23, 10.23]
		}
		if (frame.can_id== 0x712)
        {
            vehicleData.read_wheel_spd_fl = byte_handling(frame.data[2], frame.data[3]) * 0.03125; //[0, 511.96875]
            vehicleData.read_wheel_spd_fr = byte_handling(frame.data[0], frame.data[1]) * 0.03125; //[0, 511.96875]
            vehicleData.read_wheel_spd_rl = byte_handling(frame.data[6], frame.data[7]) * 0.03125; //[0, 511.96875]
            vehicleData.read_wheel_spd_rr = byte_handling(frame.data[4], frame.data[5]) * 0.03125; //[0, 511.96875]
        }
        if (frame.can_id == 0x713)
        {
            vehicleData.read_lat_accel_ = (byte_handling(frame.data[0], frame.data[1]) - 1023) * 0.01; //[-10.23, 10.23]
            vehicleData.read_yaw_rate_ = (byte_handling(frame.data[3], frame.data[4]) - 4095) * 0.01;  //[-40.95, 40.94]
            vehicleData.read_BRK_CYLINDER_ = byte_handling(frame.data[6], frame.data[7]) * 0.1;  //[0, 409.4]
        }
        if (frame.can_id == 0x720)
        {
            vehicleData.gear_cur = frame.data[0]&0b00001111; //current gear value[P:0, R:7, N:6, D:5] of the vehicle
            vehicleData.gear_cmd = frame.data[1]&0b00001111;  //gear command vaule[P:0, R:7, N:6, D:5] of user
            vehicleData.gear_control_status=frame.data[2]&0b00001111; // satatus of current gear controller
        }

        if(vehicleData.EPS_Control_Status==1 || vehicleData.EPS_Control_Status==2)
            safety_driver_status.vehicle_EPS_approved=true;
        else
            safety_driver_status.vehicle_EPS_approved=false;

        if(vehicleData.ACC_Control_Status==1 || vehicleData.ACC_Control_Status==2)
            safety_driver_status.vehicle_ACC_approved=true;
        else
            safety_driver_status.vehicle_ACC_approved=false;

        if(vehicleData.EPS_En_Status==1)
            safety_driver_status.vehicle_EPS_Status=true;
        else
            safety_driver_status.vehicle_EPS_Status=false;
        
        if(vehicleData.ACC_En_Status==1)
            safety_driver_status.vehicle_ACC_Status=true;
        else
            safety_driver_status.vehicle_ACC_Status=false;

        status_to_ros_msg(safety_driver_status, safety_driver_status_msg);
        vehicle_data_to_ros_msg(vehicleData, vehicle_data_msg);
        // vehicle_data_to_pvd_msg(vehicleData, pvd_data_msg);

        pubSafetyDriverStatus.publish(safety_driver_status_msg);
        pubVehicleData.publish(vehicle_data_msg);
        // pubPVD_.publish(pvd_data_msg);
        
        inform_the_user_about_success_once();

	}


	if (close(s) < 0) {
		perror("Close");
		return 1;
	}

	return 0;
}