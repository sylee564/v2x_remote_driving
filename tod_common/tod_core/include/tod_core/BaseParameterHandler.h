#pragma once
#include<iostream>
#include "ros/ros.h"
#include "ros/package.h"
#include <thread>
#include <atomic>

class BaseParameterHandler {
public:
    explicit BaseParameterHandler(const std::string& name);
    virtual ~BaseParameterHandler() {_alive = false; _checkActualityThread.join();}
    BaseParameterHandler(BaseParameterHandler&&) = default; // required due to user-defined dtor
    BaseParameterHandler& operator=(BaseParameterHandler&&) = default;
    BaseParameterHandler(const BaseParameterHandler&) = default; // required due to user-defined move ctor
    BaseParameterHandler& operator=(const BaseParameterHandler&) = default;

    virtual void load_parameters() = 0;
    bool vehicle_id_has_changed() const;
    void update_vehicle_id();
    void parameters_updated() { _paramsUpToDate = true; }
    std::string get_path_to_config_files() const;
    std::string get_vehicle_id() const { return _vehicleID; }

private:
    std::string _name;
    std::string _vehicleID;
    ros::NodeHandle _nh;
    std::atomic<bool> _paramsUpToDate{false};
    bool _alive{true}, _parametersLoadedFromWorkspace{false};
    std::thread _checkActualityThread;
};
