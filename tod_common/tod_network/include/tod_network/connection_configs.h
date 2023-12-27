#pragma once
#include <string>

namespace tod_network {

static const int RX_MQTT = 1883; // same for operator and vehicle
static const std::string OperPC = "172.30.1.13";

enum VehiclePorts { // only what is sent via udp
    RX_PRIMARYCONTROL_COMMAND = 7001,
    RX_SECONDARY_COMMAND = 7003,
    RX_VIDEO_RTSP = 8554,
};


enum OperatorPorts { // only what is sent via udp
    RX_LIDAR_OBJECTLIST = 5000,
    RX_VEHICLESTATE_VEHICLEDATA = 5001,
    RX_VEHICLE_PVD = 5002,
    // 50003
    RX_VEHICLESTATE_ODOMETRY = 5005,
    RX_VEHICLESTATE_GPS = 5006,
    RX_BITRATE_PREDICTIONS = 5011,
    RX_LIDAR_DATA_RANGE_FROM = 5010,
    RX_LIDAR_DATA_RANGE_TO = 5199,
    RX_LIDAR_OBJECTS_RANGE_FROM = 5020,
    RX_LIDAR_OBJECTS_RANGE_TO = 5299,
    RX_LIDAR_OBJECT_MARKER_RANGE_FROM = 5030,
    RX_LIDAR_OBJECT_MARKER_RANGE_TO = 5399,
    TX_CONTROL_COMMAND = 7000,
};

namespace MqttTopics {
static const std::string DesiredVideoConfig{"/Operator/Video/DesiredVideoConfig"};
static const std::string ActualVideoConfig{"/Vehicle/Video/ActualVideoConfig"};
static const std::string DesiredBitrateConfig{"/Operator/Video/DesiredBitrateConfig"};
static const std::string ActualBitrateConfig{"/Vehicle/Video/ActualBitrateConfig"};
};

}; // namespace tod_network