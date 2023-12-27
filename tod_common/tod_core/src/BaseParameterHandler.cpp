#include "tod_core/BaseParameterHandler.h"

BaseParameterHandler::BaseParameterHandler(const std::string &name) : _name{name} {
    update_vehicle_id();
    // Check in thread if params are up to date (Client gets informed if update is not performed)
    _checkActualityThread = std::thread([=] {
        ros::Rate r(0.2);
        while (ros::ok() && _alive) {
            r.sleep();
            if (vehicle_id_has_changed()) {
                _paramsUpToDate = false;
            }
            if (!_paramsUpToDate) {
                ROS_ERROR_STREAM(ros::this_node::getName() << " is using outdated " << _name << "Parameters!");
            }
        }
    });
}

bool BaseParameterHandler::vehicle_id_has_changed() const {
    std::string newId{""};
    if (!_nh.getParam("/vehicleID", newId)) {
        return true;
    }
    return (newId != _vehicleID);
}

void BaseParameterHandler::update_vehicle_id() {
    ros::Rate r(1);
    while (!_nh.getParam("/vehicleID", _vehicleID) && ros::ok()) {
        ROS_ERROR("%s: %sParameters waiting for parameter '/vehicleID' to be set",
                  ros::this_node::getName().c_str(), _name.c_str());
        r.sleep();
    }
    _paramsUpToDate = false;
}

std::string BaseParameterHandler::get_path_to_config_files() const {
    return ros::package::getPath("tod_vehicle_config") + "/vehicle_config/" + _vehicleID;
}
