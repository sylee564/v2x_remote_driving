#pragma once
#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <tod_helper/vehicle/Model.h>
#include "tod_core/BaseParameterHandler.h"
#include "tod_core/YamlLoader.h"

namespace tod_core {

class VehicleParameters : public BaseParameterHandler {
public:
    explicit VehicleParameters(ros::NodeHandle& nh);
    void load_parameters() override;

    float get_mass() const;
    float get_yaw_inertia() const;
    float get_distance_front_axle() const;
    float get_distance_rear_axle() const;
    float get_distance_front_bumper() const;
    float get_distance_rear_bumper() const;
    float get_width() const;
    float get_height() const;
    float get_track_width() const;
    float get_cornering_force_front() const;
    float get_cornering_force_rear() const;
    float get_max_rwa_deg() const;
    float get_max_swa_deg() const;
    float get_max_rwa_rad() const;
    float get_max_swa_rad() const;
    float get_wheel_base() const;
    float compute_swa_from(const float rwa) const;
    float compute_rwa_from(const float swa) const;

private:
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

} // namespace tod_core
