#pragma once
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <ros/console.h>
#include <tod_helper/vehicle/Model.h>

namespace tod_helper::Vehicle {

class Parameters {
public:
    Parameters();
    explicit Parameters(ros::NodeHandle& nh);
    void load_from_parameter_workspace(ros::NodeHandle& nh);

    float get_mass();
    float get_yaw_inertia();
    float get_distance_front_axle();
    float get_distance_rear_axle();
    float get_distance_front_bumper();
    float get_distance_rear_bumper();
    float get_width();
    float get_height();
    float get_track_width();
    float get_cornering_force_front();
    float get_conrering_force_rear();
    float get_max_rwa_deg();
    float get_max_swa_deg();
    float get_max_rwa_rad();
    float get_max_swa_rad();
    float get_wheel_base();

private:
    bool _parametersLoadedFromWorkspace{false};
    float _mass; // Vehicle mass in kg
    float _yaw_inertia; // Yaw inertia of the vehicle in kilogram*meter^2
    float _distance_front_axle; // Distance between CoM and front axle in meters
    float _distance_rear_axle;  // Distance between CoM and front axle in meters
    float _width_edge_to_edge; //Width from tips of side-mirrors in meters
    float _cornering_force_front; // Front cornering force in Newton
    float _cornering_force_rear; // Rear cornering force in Newton
    float _maximum_road_wheel_angle;
    float _maximum_steering_wheel_angle;
    float _height;
    float _track_width;
    float _distance_front_bumper;
    float _distance_rear_bumper;
};

inline Parameters::Parameters() { }

inline Parameters::Parameters(ros::NodeHandle& nh) {
    load_from_parameter_workspace(nh);
}

inline void Parameters::load_from_parameter_workspace(ros::NodeHandle& nh) {
    if (!nh.getParam(ros::this_node::getName() + "/mass", _mass))
        ROS_ERROR("%s: Could not get parameter mass - using %f",
                  ros::this_node::getName().c_str(), _mass);

    if (!nh.getParam(ros::this_node::getName() + "/yaw_inertia", _yaw_inertia))
        ROS_ERROR("%s: Could not get parameter yaw_inertia - using %f",
                  ros::this_node::getName().c_str(), _yaw_inertia);

    if (!nh.getParam(ros::this_node::getName() + "/distance_front_axle", _distance_front_axle))
        ROS_ERROR("%s: Could not get parameter distance_front_axle - using %f",
                  ros::this_node::getName().c_str(), _distance_front_axle);

    if (!nh.getParam(ros::this_node::getName() + "/distance_rear_axle", _distance_rear_axle))
        ROS_ERROR("%s: Could not get parameter distance_rear_axle - using %f",
                  ros::this_node::getName().c_str(), _distance_rear_axle);

    if (!nh.getParam(ros::this_node::getName() + "/width_edge_to_edge", _width_edge_to_edge))
        ROS_ERROR("%s: Could not get parameter width_edge_to_edge - using %f",
                  ros::this_node::getName().c_str(), _width_edge_to_edge);

    if (!nh.getParam(ros::this_node::getName() + "/track_width", _track_width))
        ROS_ERROR("%s: Could not get parameter track_width - using %f",
                  ros::this_node::getName().c_str(), _track_width);

    if (!nh.getParam(ros::this_node::getName() + "/cornering_force_front", _cornering_force_front))
        ROS_ERROR("%s: Could not get parameter cornering_force_front - using %f",
                  ros::this_node::getName().c_str(), _cornering_force_front);

    if (!nh.getParam(ros::this_node::getName() + "/cornering_force_rear", _cornering_force_rear))
        ROS_ERROR("%s: Could not get parameter cornering_force_rear - using %f",
                  ros::this_node::getName().c_str(), _cornering_force_rear);

    if (!nh.getParam(ros::this_node::getName() + "/maximum_road_wheel_angle", _maximum_road_wheel_angle))
        ROS_ERROR("%s: Could not get maximum_road_wheel_angle - using %f deg",
                  ros::this_node::getName().c_str(), tod_helper::Vehicle::Model::rad2deg(_maximum_road_wheel_angle));

    if (!nh.getParam(ros::this_node::getName() + "/maximum_steering_wheel_angle", _maximum_steering_wheel_angle))
        ROS_ERROR("%s: Could not get maximum_steering_wheel_angle - using %f deg", ros::this_node::getName().c_str(),
                  tod_helper::Vehicle::Model::rad2deg(_maximum_steering_wheel_angle));

    if (!nh.getParam(ros::this_node::getName() + "/height", _height))
        ROS_ERROR("%s: Could not get height - using %f",
                  ros::this_node::getName().c_str(), _height);

    if (!nh.getParam(ros::this_node::getName() + "/distance_front_bumper", _distance_front_bumper))
        ROS_ERROR("%s: Could not get parameter distance_front_bumper - using %f",
                  ros::this_node::getName().c_str(), _distance_front_bumper);

    if (!nh.getParam(ros::this_node::getName() + "/distance_rear_bumper", _distance_rear_bumper))
        ROS_ERROR("%s: Could not get parameter distance_rear_axle - using %f",
                  ros::this_node::getName().c_str(), _distance_rear_bumper);

    _parametersLoadedFromWorkspace = true;
}

inline float Parameters::get_mass() {
    if (!_parametersLoadedFromWorkspace)
        ROS_ERROR_STREAM(ros::this_node::getName()
                         << ": Vehicle parameters not loades from Workspace, node is using default parameters!");
    return _mass;
}

inline float Parameters::get_wheel_base() {
    if (!_parametersLoadedFromWorkspace)
        ROS_ERROR_STREAM(ros::this_node::getName()
                         << ": Vehicle parameters not loades from Workspace, node is using default parameters!");
    return _distance_front_axle + _distance_rear_axle;
}

inline float Parameters::get_yaw_inertia() {
    if (!_parametersLoadedFromWorkspace)
        ROS_ERROR_STREAM(ros::this_node::getName()
                         << ": Vehicle parameters not loades from Workspace, node is using default parameters!");
    return _yaw_inertia;
}

inline float Parameters::get_distance_front_axle() {
    if (!_parametersLoadedFromWorkspace)
        ROS_ERROR_STREAM(ros::this_node::getName()
                         << ": Vehicle parameters not loades from Workspace, node is using default parameters!");
    return _distance_front_axle;
}

inline float Parameters::get_distance_rear_axle() {
    if (!_parametersLoadedFromWorkspace)
        ROS_ERROR_STREAM(ros::this_node::getName()
                         << ": Vehicle parameters not loades from Workspace, node is using default parameters!");
    return _distance_rear_axle;
}

inline float Parameters::get_distance_front_bumper() {
    if (!_parametersLoadedFromWorkspace)
        ROS_ERROR_STREAM(ros::this_node::getName()
                         << ": Vehicle parameters not loades from Workspace, node is using default parameters!");
    return _distance_front_bumper;
}

inline float Parameters::get_distance_rear_bumper() {
    if (!_parametersLoadedFromWorkspace)
        ROS_ERROR_STREAM(ros::this_node::getName()
                         << ": Vehicle parameters not loades from Workspace, node is using default parameters!");
    return _distance_rear_bumper;
}

inline float Parameters::get_width() {
    if (!_parametersLoadedFromWorkspace)
        ROS_ERROR_STREAM(ros::this_node::getName()
                         << ": Vehicle parameters not loades from Workspace, node is using default parameters!");
    return _width_edge_to_edge;
}

inline float Parameters::get_height() {
    if (!_parametersLoadedFromWorkspace)
        ROS_ERROR_STREAM(ros::this_node::getName()
                         << ": Vehicle parameters not loades from Workspace, node is using default parameters!");
    return _height;
}

inline float Parameters::get_track_width() {
    if (!_parametersLoadedFromWorkspace)
        ROS_ERROR_STREAM(ros::this_node::getName()
                         << ": Vehicle parameters not loades from Workspace, node is using default parameters!");
    return _track_width;
}

inline float Parameters::get_cornering_force_front() {
    if (!_parametersLoadedFromWorkspace)
        ROS_ERROR_STREAM(ros::this_node::getName()
                         << ": Vehicle parameters not loades from Workspace, node is using default parameters!");
    return _cornering_force_front;
}

inline float Parameters::get_conrering_force_rear() {
    if (!_parametersLoadedFromWorkspace)
        ROS_ERROR_STREAM(ros::this_node::getName()
                         << ": Vehicle parameters not loades from Workspace, node is using default parameters!");
    return _cornering_force_rear;
}

inline float Parameters::get_max_rwa_deg() {
    if (!_parametersLoadedFromWorkspace)
        ROS_ERROR_STREAM(ros::this_node::getName()
                         << ": Vehicle parameters not loades from Workspace, node is using default parameters!");
    return tod_helper::Vehicle::Model::rad2deg(_maximum_road_wheel_angle);
}

inline float Parameters::get_max_swa_deg() {
    if (!_parametersLoadedFromWorkspace)
        ROS_ERROR_STREAM(ros::this_node::getName()
                         << ": Vehicle parameters not loades from Workspace, node is using default parameters!");
    return tod_helper::Vehicle::Model::rad2deg(_maximum_steering_wheel_angle);
}

inline float Parameters::get_max_rwa_rad() {
    if (!_parametersLoadedFromWorkspace)
        ROS_ERROR_STREAM(ros::this_node::getName()
                         << ": Vehicle parameters not loades from Workspace, node is using default parameters!");
    return _maximum_road_wheel_angle;
}

inline float Parameters::get_max_swa_rad() {
    if (!_parametersLoadedFromWorkspace)
        ROS_ERROR_STREAM(ros::this_node::getName()
                         << ": Vehicle parameters not loades from Workspace, node is using default parameters!");
    return _maximum_steering_wheel_angle;
}
} // namespace tod_helper::Vehicle