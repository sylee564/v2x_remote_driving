#include <ros/ros.h>
#include <math.h>
#include <iostream>
#include <string.h>
#include <vector>
#include "vehicle_tyep.h"

#include "tod_msgs/VehicleEnums.h"

//subscribe topic msgs header
#include "tod_msgs/VehicleData.h"
#include "tod_msgs/SafetyDriverStatus.h"
#include "pvd_msgs/PVD_data.h"
#include "tod_msgs/PVD_data.h"
#include "novatel_oem7_msgs/BESTPOS.h"
#include "novatel_oem7_msgs/HEADING2.h"

#include <tod_helper/vehicle/Model.h>


typedef struct PVDdata{
    //probe ID
    std::string vehicle_name;
    std::string vehicle_id;

    //vehicle type
    uint32_t vehicle_type;

    //vehicle status
    int steering_wheel;
    float acceleration;
    
    uint8_t velocity;
    uint8_t gear_status;

    uint8_t lat_approved;   //Only kona, Ioniq phev
    uint8_t long_approved;  //Only kona, Ioniq phev
    uint8_t vehicle_mode;
    uint8_t vehicle_mile;

    //GNSS data
    double latitude;
    double longitude;
    float elevation;
    float heading;

    //sensor status
    uint8_t gnss_status;
    std::vector<uint8_t> camera_status;
    std::vector<uint8_t> lidar_status;
    std::vector<uint8_t> radar_status;

    PVDdata() :
        vehicle_name(""), vehicle_id(""), vehicle_type(0), steering_wheel(0), 
        acceleration(0), velocity(0), latitude(0.0),  longitude(0.0), elevation(0.0),
        heading(0.0), lat_approved(0), long_approved(0), vehicle_mode(0), vehicle_mile(0), gnss_status(0)
        {   
            camera_status.emplace_back(0xff);
            lidar_status.emplace_back(0xff);
            radar_status.emplace_back(0xff);
        }
} tPVDdata;

tPVDdata temporaryPVDdata;

// Callback vehicle data
void callback_vehicle_data(const tod_msgs::VehicleData::ConstPtr &vechie_data_msg) {

    temporaryPVDdata.steering_wheel=tod_helper::Vehicle::Model::rad2deg(vechie_data_msg->steeringWheelAngle);
    temporaryPVDdata.acceleration=vechie_data_msg->linearAcceleration.y;
    temporaryPVDdata.velocity =tod_helper::Vehicle::Model::mps2kph(vechie_data_msg->vehicle_velocity);

    switch (vechie_data_msg->gearPosition) {
        case GEARPOSITION_PARK: temporaryPVDdata.gear_status = 0; break;
        case GEARPOSITION_REVERSE: temporaryPVDdata.gear_status = 1; break;
        case GEARPOSITION_NEUTRAL: temporaryPVDdata.gear_status = 2; break;
        case GEARPOSITION_DRIVE: temporaryPVDdata.gear_status = 3; break;
    }

}


// Callback Vehicle Mode
void callback_safety_driver_status(const tod_msgs::SafetyDriverStatus::ConstPtr &safety_driver_status_msg){
    
    temporaryPVDdata.lat_approved = safety_driver_status_msg->vehicle_EPS_approved;
    temporaryPVDdata.long_approved = safety_driver_status_msg->vehicle_ACC_approved;

    if(safety_driver_status_msg->vehicle_EPS_Status && safety_driver_status_msg->vehicle_ACC_Status){
        temporaryPVDdata.vehicle_mode=1;
    }
    else{
        temporaryPVDdata.vehicle_mode=0;
    }
}

// Callback Novatel GPS
void callback_gps_status(const novatel_oem7_msgs::BESTPOS::ConstPtr &gps_msg){
    temporaryPVDdata.latitude=gps_msg->lat;
    temporaryPVDdata.longitude=gps_msg->lon;
    temporaryPVDdata.elevation=gps_msg->hgt;
}

// Callback Novatel Heading
void callback_heading_status(const novatel_oem7_msgs::HEADING2::ConstPtr &heading_msg){
    temporaryPVDdata.heading=heading_msg->heading;
}

/*
void callback_sensor_diagnotics(const diagnotics_msgs:: &diagnotic_msg) {
    temporaryPVDdata.camera_status.clear();
    temporaryPVDdata.lidar_status.clear();
    temporaryPVDdata.radar_status.clear();
    // insert diagnotics_msg to temporary PVD data


}
*/

//Temporary PVD structure to ros msg(PVD msg)
void temporary_data_to_spring_msg(const tPVDdata &tempData, pvd_msgs::PVD_data &msg) {
    

    msg.vehicle_name = tempData.vehicle_name;
    msg.vehicle_id = tempData.vehicle_id;
    msg.vehicle_type = tempData.vehicle_type;

    msg.steering_wheel = tempData.steering_wheel;
    msg.acceleration = tempData.acceleration;
    msg.velocity = tempData.velocity;
    msg.gear_status = tempData.gear_status;

    msg.latitude = tempData.latitude;
    msg.longitude = tempData.longitude;
    msg.elevation = tempData.elevation;

    msg.heading = tempData.heading;

    msg.lat_approved = tempData.lat_approved;
    msg.long_approved = tempData.long_approved;

    msg.vehicle_mode = tempData.vehicle_mode;

    msg.gnss_status = tempData.gnss_status;
    msg.camera_status = tempData.camera_status;
    msg.lidar_status = tempData.lidar_status;
    msg.radar_status = tempData.radar_status;

}

void temporary_data_to_pvd_msg(const tPVDdata &tempData, tod_msgs::PVD_data &msg) {
    
    static int ctrSend{0};

    msg.header.seq = ++ctrSend;
    msg.header.stamp = ros::Time::now();

    msg.vehicle_name = tempData.vehicle_name;
    msg.vehicle_id = tempData.vehicle_id;
    msg.vehicle_type = tempData.vehicle_type;

    msg.steering_wheel = tempData.steering_wheel;
    msg.acceleration = tempData.acceleration;
    msg.velocity = tempData.velocity;
    msg.gear_status = tempData.gear_status;

    msg.latitude = tempData.latitude;
    msg.longitude = tempData.longitude;
    msg.elevation = tempData.elevation;

    msg.heading = tempData.heading;

    msg.lat_approved = tempData.lat_approved;
    msg.long_approved = tempData.long_approved;

    msg.vehicle_mode = tempData.vehicle_mode;

    msg.gnss_status = tempData.gnss_status;
    msg.camera_status = tempData.camera_status;
    msg.lidar_status = tempData.lidar_status;
    msg.radar_status = tempData.radar_status;

}

float dist=0.0;
//calculate vehicle mileage
void calculate_vehicle_mile(){
    static ros::Time prevTime = ros::Time::now();
    float velocit_kph = temporaryPVDdata.velocity;
    float velocit_mps = tod_helper::Vehicle::Model::kph2mps(velocit_kph);
    
    //Integrate distance
    ros::Duration dt = ros::Time::now() - prevTime;
    prevTime = ros::Time::now();
    dist = dist + dt.toSec() * velocit_mps;

    if(dist-ceil(dist) > 0){
        temporaryPVDdata.vehicle_mile+=1;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "VehicleDataToPVD");
	ros::NodeHandle n;
    //vehicle data subscriber 
	ros::Subscriber subVehicleData=n.subscribe("/Vehicle/VehicleBridge/vehicle_data",
            1, callback_vehicle_data);
    ros::Subscriber subSafetyDriverStatus = n.subscribe("/Vehicle/VehicleBridge/safety_driver_status",
            1, callback_safety_driver_status);

    // Novatel GNSS subscriber
    ros::Subscriber subGpsStatus = n.subscribe("/novatel/oem7/bestpos", 1, callback_gps_status);
    ros::Subscriber subHeadingStatus = n.subscribe("/novatel/oem7/heading2", 1, callback_heading_status);
    

    // PVD msg publication
    ros::Publisher pubPVD_=n.advertise<pvd_msgs::PVD_data>("PVD_data", 1);
    ros::Publisher pubPVD_1=n.advertise<tod_msgs::PVD_data>("PVD", 1);

    ros::Rate loopRate(100);
    ros::Duration(1).sleep();

    std::string vehicle_name{"kona"};
    std::string vehicle_id{"123°¡4567"};

    if (!n.getParam(ros::this_node::getName() + "/vehicleName", vehicle_name))
        ROS_ERROR_STREAM(ros::this_node::getName() << ": Could not get param /vehicleName - using ");

    if (!n.getParam(ros::this_node::getName() + "/vehicleID", vehicle_id))
        ROS_ERROR_STREAM(ros::this_node::getName() << ": Could not get param /vehicleID - using ");
    
    // if (!n.getParam(ros::this_node::getName() + "/vehicleType", vehicle_type))
    //     ROS_ERROR_STREAM(ros::this_node::getName() << ": Could not get param /vehicleType - using ");

    pvd_msgs::PVD_data pvd_data_msg;
    tod_msgs::PVD_data pvd_msg;
    temporaryPVDdata.vehicle_name = vehicle_name;
    temporaryPVDdata.vehicle_id = vehicle_id;
    temporaryPVDdata.vehicle_type = CAR;    // refer to vehicle_type.h  

    /* if there is sensors, vector clear */
    // temporaryPVDdata.camera_status.clear();
    // temporaryPVDdata.lidar_status.clear();
    // temporaryPVDdata.radar_status.clear();
    
    temporaryPVDdata.gnss_status = 1;
    temporaryPVDdata.camera_status.clear();
    temporaryPVDdata.camera_status.emplace_back(1);
    temporaryPVDdata.camera_status.emplace_back(1);
    temporaryPVDdata.camera_status.emplace_back(1);
    temporaryPVDdata.camera_status.emplace_back(1);

    while(ros::ok()){
        ros::spinOnce();
        calculate_vehicle_mile();
        temporary_data_to_spring_msg(temporaryPVDdata, pvd_data_msg);
        temporary_data_to_pvd_msg(temporaryPVDdata, pvd_msg);
        

        pubPVD_.publish(pvd_data_msg);
        pubPVD_1.publish(pvd_msg);
        loopRate.sleep();
    }


	return 0;
}