#include "VehicleManagerXavier.h"

VehicleManagerXavier::VehicleManagerXavier() 
: ip_addr_checker(new IPv4ValidityChecker){
    init_rosthread_msg_data();
}

void VehicleManagerXavier::create_and_run_ros_thread() {
    its_thread = std::thread(&rosloop_vehicle::ros_run_spin_loop,
                             &its_ros_loop, "VehicleManagerXavierNode", "status_msg",
                             &its_status_msg, &ros_terminated);
}

void VehicleManagerXavier::wait_for_ros_thread_to_join() {
    its_thread.join();
}

bool VehicleManagerXavier::check_ip_addr_validity(const std::string& ip_addr) {
    return ip_addr_checker->validate(ip_addr);
}

bool VehicleManagerXavier::create_mqtt_client(const std::string& client_id,
    const std::string& ip_addr_broker) {
    if ( !check_ip_addr_validity(ip_addr_broker) ) {
        std::cerr << "In VehicleManager.cpp (24): Broker IP address format is not valid\n";
        return false;
    }
    its_client.reset(new tod_network::MqttClientTemplated<VehicleManagerXavier>(ip_addr_broker, client_id));
    if (its_client->is_connected()) {
        return true;
    } else {
        std::cerr << "In VehicleManager.cpp (110): Could not reach provided broker\n";
        return false;
    }
}

void VehicleManagerXavier::set_mqtt_callback_to_topic(const std::string& mqtt_topic_name) {
    its_client->subscribe(mqtt_topic_name, 1,
        &VehicleManagerXavier::callback_tod_network_client, this);
}

void VehicleManagerXavier::callback_tod_network_client(mqtt::const_message_ptr msg) {
    ros::serialization::IStream stream((uint8_t*) msg->get_payload_str().c_str(),
        msg->get_payload().length());
    tod_msgs::Status tmp;
    ros::serialization::Serializer<tod_msgs::Status>::read(stream, tmp);
    set_tod_status_of_its_status_msg(tmp);
    set_rosthread_msg_operator_part(tmp);

    // pub_its_status_msg_to_broker();
    if ( number_received_packages >= INT_MAX ) {
        number_received_packages = 0;
    }
    ++number_received_packages;
 }


void VehicleManagerXavier::set_mqtt_pub_topic_to(const std::string& topic) {
    its_mqtt_response_topic = topic;
}

void VehicleManagerXavier::pub_its_status_msg_to_broker() {
    its_mutex.lock();
    ros::SerializedMessage rosSer = ros::serialization::serializeMessage
        <tod_msgs::Status>(its_status_msg);
    its_client->publish(its_mqtt_response_topic, 1, (char*) rosSer.message_start,
        rosSer.num_bytes);
    its_mutex.unlock();
}

void VehicleManagerXavier::init_rosthread_msg_data() {
    its_mutex.lock();
    its_status_msg.tod_status = tod_msgs::Status::TOD_STATUS_IDLE;
    its_status_msg.vehicle_EPS_approved = false;
    its_status_msg.vehicle_ACC_approved = false;
    its_status_msg.vehicle_EPS_Status = false;
    its_status_msg.vehicle_ACC_Status = false;
    its_status_msg.vehicle_emergency_stop_released = false;
    its_status_msg.vehicle_PC_ip_address = "127.0.0.1";
    its_status_msg.vehicle_control_mode =
        tod_msgs::Status::CONTROL_MODE_DIRECT;
    its_status_msg.operator_video_mode =
        tod_msgs::Status::VIDEO_RATE_CONTROL_MODE_SINGLE;
    its_mutex.unlock();
    update_vehicle_time_stamp();
}

void VehicleManagerXavier::set_tod_status_of_its_status_msg(const tod_msgs::Status& msg) {
    its_mutex.lock();
    its_status_msg.tod_status = msg.tod_status;
    its_mutex.unlock();
    update_vehicle_time_stamp();
}

void VehicleManagerXavier::set_rosthread_msg_operator_part(const tod_msgs::Status& msg) {
    its_mutex.lock();
    its_status_msg.operator_header = msg.operator_header;
    its_status_msg.operator_control_mode = msg.operator_control_mode;
    its_status_msg.operator_video_mode = msg.operator_video_mode;
    its_status_msg.operator_device_ip_address = msg.operator_device_ip_address;
    its_status_msg.operator_PC_ip_address = msg.operator_PC_ip_address;
    its_status_msg.operator_device_port = msg.operator_device_port;
    its_status_msg.vehicle_control_mode = msg.operator_control_mode;
    its_status_msg.oper_v2x_header = msg.oper_v2x_header;
    its_status_msg.stream_flag = msg.stream_flag;
    its_status_msg.gear = msg.gear;
    its_mutex.unlock();
    update_vehicle_time_stamp();
}


int VehicleManagerXavier::num_rec_mqtt_packages() {
    return number_received_packages;
}

void VehicleManagerXavier::change_tod_status_msg_if_disconnected(bool disconnection_detected) {
    if (disconnection_detected) {
        its_mutex.lock();
        its_status_msg.tod_status = tod_msgs::Status::TOD_STATUS_IDLE;
        its_mutex.unlock();
        update_vehicle_time_stamp();
    }
}

tod_msgs::Status VehicleManagerXavier::get_its_status_msg() {
    return its_status_msg;
}

void VehicleManagerXavier::update_vehicle_time_stamp() {
    ros::Time::init();
    its_mutex.lock();
    its_status_msg.vehicle_header.stamp = ros::Time::now();
    its_mutex.unlock();
}

// void VehicleManager::callback_nav_status(const std_msgs::String& nav_status ) {
//     const std::lock_guard<std::mutex> lock(its_mutex);
//     its_status_msg.vehicle_nav_status = nav_status.data;
// }

// void VehicleManager::callback_pos_type(const std_msgs::String& pos_type) {
//     const std::lock_guard<std::mutex> lock(its_mutex);
//     its_status_msg.vehicle_gps_pos_type = pos_type.data;
// }
