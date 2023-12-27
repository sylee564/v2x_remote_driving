#include <ros/ros.h>
#include <QApplication>
#include "VehicleManager.h"
#include <ros/package.h>

int main(int argc, char **argv) {
    QApplication a(argc, argv);
    ros::init(argc, argv, "VehicleManager");
    ros::NodeHandle nh;
    std::string pathToYamlFile = ros::package::getPath("tod_manager") + "/config/V2X_Config.yaml";
    VehicleManager vehicleManager(pathToYamlFile, "ConfigNodes");
    ros::Subscriber pvd_data_subscriber =
        nh.subscribe("/Vehicle/VehicleBridge/PVD", 5, &VehicleManager::callback_pvd_data,
            &vehicleManager);
    ros::Subscriber safety_driver_status_subscriber =
        nh.subscribe("/Vehicle/VehicleBridge/safety_driver_status", 5, &VehicleManager::callback_safety_driver_status,
            &vehicleManager);
    ros::Subscriber control_command_subscriber =
        nh.subscribe("/Vehicle/CommandCreation/control_cmd_data", 5, &VehicleManager::callback_control_command_data,
            &vehicleManager);
    vehicleManager.create_and_run_ros_thread();
    return a.exec();
}
