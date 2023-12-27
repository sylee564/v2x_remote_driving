#include "ros/ros.h"
#include "VehicleManagerXavier.h"
#include <iostream>
#include <fstream>
#include <string>
#include <ostream>
#include <streambuf>

void init_vehicle_manager(VehicleManagerXavier* veh_man_xavier, const std::string &ip_addr_broker) {
    veh_man_xavier->create_and_run_ros_thread();
    bool t1 = veh_man_xavier->create_mqtt_client("vehicle_client2", ip_addr_broker);
    veh_man_xavier->set_mqtt_callback_to_topic("Operator/Manager/status_msg");
    // veh_man_xavier->set_mqtt_pub_topic_to("Vehicle/Manager/status_msg");
}

void wait_until_ros_is_shutdown(VehicleManagerXavier* veh_man_xavier) {
    veh_man_xavier->wait_for_ros_thread_to_join();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "VehicleManagerXavier");
    ros::NodeHandle nh;

    std::string ip_addr_broker{"127.0.0.1"};
    if (!nh.getParam(ros::this_node::getName() + "/IpAddress", ip_addr_broker))
        ROS_ERROR_STREAM(ros::this_node::getName() << ": Could not get param /IpAddress - using ");
    std::cout << "ip : "<< ip_addr_broker <<std::endl;
    VehicleManagerXavier vehicleManagerXavier;
    init_vehicle_manager(&vehicleManagerXavier, ip_addr_broker);


    wait_until_ros_is_shutdown(&vehicleManagerXavier);
    return 0;
}
