#include "tod_core/VehicleParameters.h"

namespace tod_core {

VehicleParameters::VehicleParameters(ros::NodeHandle& nh) : BaseParameterHandler("Vehicle") {
    load_parameters(); // get first Parameter Set
}

void VehicleParameters::load_parameters() {
    update_vehicle_id();
    YamlLoader loader;
    if (!loader.load_from_path(get_path_to_config_files() + "/vehicle-params.yaml")) {
        return;
    }

    _mass = loader.get_param<float>("mass");
    _yaw_inertia = loader.get_param<float>("yaw_inertia");
    _distance_front_axle = loader.get_param<float>("distance_front_axle");
    _distance_rear_axle = loader.get_param<float>("distance_rear_axle");
    _width_edge_to_edge = loader.get_param<float>("width_edge_to_edge");
    _track_width = loader.get_param<float>("track_width");
    _cornering_force_front = loader.get_param<float>("cornering_force_front");
    _cornering_force_rear = loader.get_param<float>("cornering_force_rear");
    _maximum_road_wheel_angle = loader.get_param<float>("maximum_road_wheel_angle");
    _maximum_steering_wheel_angle = loader.get_param<float>("maximum_steering_wheel_angle");
    _height = loader.get_param<float>("height");
    _distance_front_bumper = loader.get_param<float>("distance_front_bumper");
    _distance_rear_bumper = loader.get_param<float>("distance_rear_bumper");

    parameters_updated();
}

float VehicleParameters::get_mass() const {
    return _mass;
}

float VehicleParameters::get_wheel_base() const {
    return _distance_front_axle + _distance_rear_axle;
}

float VehicleParameters::get_yaw_inertia() const {
    return _yaw_inertia;
}

float VehicleParameters::get_distance_front_axle() const {
    return _distance_front_axle;
}

float VehicleParameters::get_distance_rear_axle() const {
    return _distance_rear_axle;
}

float VehicleParameters::get_distance_front_bumper() const {
    return _distance_front_bumper;
}

float VehicleParameters::get_distance_rear_bumper() const {
    return _distance_rear_bumper;
}

float VehicleParameters::get_width() const {
    return _width_edge_to_edge;
}

float VehicleParameters::get_height() const {
    return _height;
}

float VehicleParameters::get_track_width() const {
    return _track_width;
}

float VehicleParameters::get_cornering_force_front() const {
    return _cornering_force_front;
}

float VehicleParameters::get_cornering_force_rear() const {
    return _cornering_force_rear;
}

float VehicleParameters::get_max_rwa_deg() const {
    return tod_helper::Vehicle::Model::rad2deg(_maximum_road_wheel_angle);
}

float VehicleParameters::get_max_swa_deg() const {
    return tod_helper::Vehicle::Model::rad2deg(_maximum_steering_wheel_angle);
}

float VehicleParameters::get_max_rwa_rad() const {
    return _maximum_road_wheel_angle;
}

float VehicleParameters::get_max_swa_rad() const {
    return _maximum_steering_wheel_angle;
}

float VehicleParameters::compute_swa_from(const float rwa) const {
    return tod_helper::Vehicle::Model::rwa2swa(rwa, _maximum_steering_wheel_angle, _maximum_road_wheel_angle);
}

float VehicleParameters::compute_rwa_from(const float swa) const {
    return tod_helper::Vehicle::Model::swa2rwa(swa, _maximum_steering_wheel_angle, _maximum_road_wheel_angle);
}

} // namespace tod_core
