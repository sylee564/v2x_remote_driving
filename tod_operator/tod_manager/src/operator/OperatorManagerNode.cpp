#include <ros/ros.h>
#include <QApplication>
#include "OperatorManager.h"
#include <ros/package.h>

int main(int argc, char **argv) {
    QApplication a(argc, argv);
    ros::init(argc, argv, "OperatorManager");
    ros::NodeHandle nh;
    std::string pathToYamlFile = ros::package::getPath("tod_manager") + "/config/V2X_Config.yaml";
    OperatorManager operatorManager(pathToYamlFile, "ConfigNodes");
    ros::Subscriber control_command_subscriber =
        nh.subscribe("/Operator/CommandCreation/control_cmd_data", 5, &OperatorManager::callback_control_command_data,
            &operatorManager);
    ros::Subscriber pvd_subscriber =
        nh.subscribe("/Operator/VehicleBridge/PVD", 5, &OperatorManager::callback_pvd_data,
            &operatorManager);
    operatorManager.create_and_run_ros_thread();
    return a.exec();
}
