#include <ros/ros.h>
#include "ControlCmdReceiver.h"
#include <iostream>
#include <fstream>
#include <string>
#include <ostream>
#include <streambuf>
// #include "tod_msgs/Status.h"

// static ControlCmdReceiver control_cmd_receiver;

// void init_control_cmd_receiver() {
//     control_cmd_receiver.create_and_run_ros_thread();
//     // bool t1 = veh_control_cmd->create_mqtt_client("vehicle_client2", ip_addr_broker);
// }

// void wait_until_ros_is_shutdown() {
//     control_cmd_receiver.wait_for_ros_thread_to_join();
// }

// void callback_tod_status(const tod_msgs::Status::ConstPtr &status_msg){
//     ip_addr_broker = status_msg->operator_broker_ip_address;
//     // std::cout<<"ip:"<<ip_addr_broker<<std::endl;
//     if(status_msg->tod_status == tod_msgs::Status::TOD_STATUS_IDLE){
//         if(connect_flag) connect_flag = !(control_cmd_receiver.disconnect_mqtt_client());
//     }else{
//         if(!connect_flag){
//             connect_flag = control_cmd_receiver.create_mqtt_client("kona_client2", ip_addr_broker);
//         }
//         else{
//             control_cmd_receiver.set_mqtt_callback_to_topic("Vehicle/Control/control_cmd_data");
//         }
        
//     }
// }

int main(int argc, char **argv) {
    ros::init(argc, argv, "ControlCmdReceiver");
    ros::NodeHandle nh;
    ControlCmdReceiver controlCmdReceiver;
    ros::Subscriber subNetwork = nh.subscribe("/driver_tcpip/sockets/0/rx", 10, 
                            &ControlCmdReceiver::callback_tod_network_client, &controlCmdReceiver);
    ros::Subscriber subTodStatus= nh.subscribe("/Vehicle/Manager/status_msg", 1, 
                            &ControlCmdReceiver::callback_tod_status, &controlCmdReceiver);
    
    controlCmdReceiver.create_and_run_ros_thread();
    // init_control_cmd_receiver();


    controlCmdReceiver.wait_for_ros_thread_to_join();
    return 0;
}
