#pragma once
#include <iostream>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <sstream>

class YamlLoader {
private:
    YAML::Node _node;

    template <typename ... Args>
    static bool node_has_node(const YAML::Node& node, Args&& ... args) {
        YAML::Node node_new = node_get_node(node, std::forward<Args>(args)...);
        return node_new ? true : false;
    }

    // recursive get_node
    template <typename First, typename ... Args>
    static YAML::Node node_get_node(const YAML::Node& node, First&& first, Args&& ... args) {
        YAML::Node node_new = node_get_node(node, std::forward<First>(first));
        if (!node_new) // return zombienode if not exists
            return node_new;
        return node_get_node(node_new, std::forward<Args>(args)...);
    }

    template <typename Last>
    static YAML::Node node_get_node(const YAML::Node& node, Last&& last) {
        return node[std::string(std::forward<Last>(last))];
    }

public:
    template<typename T>
    bool load_from_path(T&& path) {
        try {
            _node = YAML::LoadFile(std::forward<T>(path));
            return true;
        } catch (...) {
            ROS_ERROR_STREAM(ros::this_node::getName() << ": Vehicle Specific Parameters could not be loaded." <<
                "Check the following possible Causes: \n" <<
                "- vehicleID was not set correctly \n" <<
                "- yaml file does not follow conventions \n" <<
                "- <vehicleID>/vehicle-params.yaml does not exit \n");
            return false;
        }
    }

    template<typename T, typename ... Args>
    T get_param(Args&& ... args) {
        YAML::Node node = node_get_node(_node, std::forward<Args>(args)...);
        if (node) {
            return node.as<T>();
        } else {
            std::stringstream ss; ((ss << "/" << args), ...);
            ROS_ERROR_STREAM(ros::this_node::getName() << ": Could not find param " << ss.str());
            return T();
        }
    }

    template<typename T, typename ... Args>
    T get_opt_param(Args&& ... args) {
        YAML::Node node = node_get_node(_node, std::forward<Args>(args)...);
        if (node) {
            return node.as<T>();
        } else {
            std::stringstream ss; ((ss << "/" << args), ...);
            ROS_DEBUG_STREAM(ros::this_node::getName() << ": Could not find param " << ss.str());
            return T();
        }
    }

    template<typename ... Args>
    bool has_node(Args&& ... args) {
        return node_has_node(_node, std::forward<Args>(args)...);
    }
};
