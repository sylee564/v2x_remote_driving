#include "VehicleManager.h"

VehicleManager::VehicleManager(const std::string& pathToYamlFile,
        const std::string& searchedKey)
    : ip_addr_checker(new IPv4ValidityChecker), number_received_packages(0) {

    init_its_status_msg_data();

    // create window
    its_v2xManagerWindow = new v2xManagerWindow(pathToYamlFile, searchedKey, this);

    connect(its_v2xManagerWindow, &v2xManagerWindow::signal_on_radioBtn_Select_clicked,
            this, &VehicleManager::handle_signal_on_select_clicked);
    connect(its_v2xManagerWindow, &v2xManagerWindow::signal_on_connectBtn_Connect_clicked,
            this, &VehicleManager::handle_signal_on_connect_clicked);
    connect(its_v2xManagerWindow, &v2xManagerWindow::signal_on_connectBtn_Disconnect_clicked,
            this, &VehicleManager::handle_signal_on_disconnect_clicked);
    // connect(its_v2xManagerWindow, &v2xManagerWindow::signal_on_StartAndStopBtn_Start_clicked,
    //         this, &VehicleManager::handle_signal_on_start_clicked);
    connect(its_v2xManagerWindow, &v2xManagerWindow::signal_on_StartAndStopBtn_Stop_clicked,
            this, &VehicleManager::handle_signal_on_stop_clicked);

    connect(this, &VehicleManager::signal_Connect,
            its_v2xManagerWindow, &v2xManagerWindow::on_connectBtn_clicked);
    connect(this, &VehicleManager::signal_Start,
            its_v2xManagerWindow, &v2xManagerWindow::on_StartAndStopBtn_clicked);
    connect(this, &VehicleManager::signal_Stop,
            its_v2xManagerWindow, &v2xManagerWindow::on_StartAndStopBtn_clicked);
    connect(this, &VehicleManager::signal_Disconnect,
            its_v2xManagerWindow, &v2xManagerWindow::on_connectBtn_clicked);

    connect(this, &VehicleManager::signal_vehicle_emergency_stop_released,
            its_v2xManagerWindow, &v2xManagerWindow::change_emergency_stop_released);
    connect(this, &VehicleManager::signal_vehicle_EPS_approved,
            its_v2xManagerWindow, &v2xManagerWindow::EPS_approved);
    connect(this, &VehicleManager::signal_vehicle_ACC_approved,
            its_v2xManagerWindow, &v2xManagerWindow::ACC_approved);
    
    connect(this, &VehicleManager::signal_control_command_data,
            its_v2xManagerWindow, &v2xManagerWindow::get_control_command_data);

    connect(this, &VehicleManager::signal_pvd_data,
            its_v2xManagerWindow, &v2xManagerWindow::get_pvd_data);

    // connect(this, &VehicleManager::signal_update_nav_status,
    //         its_v2xManagerWindow, &v2xManagerWindow::set_nav_status);
    // connect(this, &VehicleManager::signal_update_gps_pos_type,
    //         its_v2xManagerWindow, &v2xManagerWindow::set_gps_pos_type);

    connect(its_v2xManagerWindow, &v2xManagerWindow::signal_control_mode_changed,
            this, &VehicleManager::set_control_mode_to_its_status_msg);
    connect(its_v2xManagerWindow, &v2xManagerWindow::signal_input_device_changed,
            this, &VehicleManager::call_input_device_change_service);
    connect(its_v2xManagerWindow, &v2xManagerWindow::signal_video_mode_changed,
            this, &VehicleManager::set_video_rate_control_mode_to_its_status_msg);
    // // connect(this, &VehicleManager::new_vehicle_timestamp,
    //         its_v2xManagerWindow, &v2xManagerWindow::update_vehicle_tx_label);
    // connect(this, &VehicleManager::new_operator_timestamp,
    //         its_v2xManagerWindow, &v2xManagerWindow::update_operator_tx_label);

    check_timer = new QTimer(this);
    connect(check_timer, &QTimer::timeout,
            this, &VehicleManager::update_gui_if_different_from_its_tod_status);
    connect(check_timer, &QTimer::timeout,
            this, &VehicleManager::update_gui_status_from_its_data);
    connect(check_timer, &QTimer::timeout,
            this, &VehicleManager::close_application_if_needed);
    // connect(check_timer, &QTimer::timeout, this, [this]{
    //         tod_msgs::Status tmpStudyLeaderWishes;
    //         if ( its_ros_operator_loop.gotNewStudyLeaderWishes(tmpStudyLeaderWishes) ) {
    //             processStudyLeaderWishes(tmpStudyLeaderWishes);
    //             printf("status : %d\n", tmpStudyLeaderWishes.tod_status);
    //             // printf("process success!!!!!!\n");
    //         }

    //     });
    check_timer->start(10);
}

VehicleManager::~VehicleManager() {
    if ( its_v2xManagerWindow != nullptr ) { delete its_v2xManagerWindow; }
    check_timer->stop();
    delete check_timer;
}
void VehicleManager::handle_signal_on_select_clicked(const uint8_t &mode_status){
    set_mode_status_of_its_status_msg(mode_status);
}


void VehicleManager::handle_signal_on_connect_clicked(const std::string &ip_addr_pc, const std::string &ip_addr_device,
        const int ip_device_port, const std::vector<uint16_t> header) {
    // establish connection
    bool successful_mqtt_client_initialization = create_mqtt_client("vehicle_client1", ip_addr_device);
    if (successful_mqtt_client_initialization) {
        set_mqtt_pub_topic_to("Vehicle/Manager/status_msg");
        set_mqtt_callback_to_topic("Operator/Manager/status_msg");
        start_frequent_mqtt_pub(100);
        set_ip_addr_of_its_status_msg(ip_addr_pc, ip_addr_device, ip_device_port);
        set_tod_status_of_its_status_msg(tod_msgs::Status::TOD_STATUS_UPLINK_ONLY);
        set_v2x_header_of_its_status_msg(header);
    }
}

void VehicleManager::handle_signal_on_disconnect_clicked() {
    set_tod_status_of_its_status_msg(tod_msgs::Status::TOD_STATUS_IDLE);
    if(its_status_msg.mode_status==1){
        its_status_msg.oper_v2x_header.clear();
    }
    else{

        its_status_msg.vehicle_v2x_header.clear();
    }
    
    if ( mqtt_pub_loop_is_running ) {
        stop_frequent_mqtt_pub();
        // tell the vehicle about the disconnection:
        pub_its_status_msg_to_broker();
    }
}

void VehicleManager::handle_signal_on_start_clicked() {
    set_tod_status_of_its_status_msg(tod_msgs::Status::TOD_STATUS_TELEOPERATION);
    
}

void VehicleManager::handle_signal_on_stop_clicked() {
    set_tod_status_of_its_status_msg(tod_msgs::Status::TOD_STATUS_UPLINK_ONLY);
}

void VehicleManager::call_input_device_change_service(const std::string& input_device) {
    tod_msgs::InputDevice input_device_request;
    input_device_request.request.input_device_directory = input_device;
    bool success =  ros::service::call("change_input_device", input_device_request);
    if ( !success ) {
        ROS_ERROR("Could not reach service server for input device change at %s",
            ros::this_node::getName().c_str());
    }
}


bool VehicleManager::check_ip_addr_validity(const std::string& ip_addr) {
    return ip_addr_checker->validate(ip_addr);
}

bool VehicleManager::create_mqtt_client(const std::string& client_id,
    const std::string& ip_addr_broker) {
    if ( !check_ip_addr_validity(ip_addr_broker) ) {
        std::cerr << "In VehicleManager.cpp (102): Broker IP address format is not valid\n";
        return false;
    }
    its_client.reset(new tod_network::MqttClientTemplated<VehicleManager>(ip_addr_broker, client_id));
    if (its_client->is_connected()) {
        return true;
    } else {
        std::cerr << "In VehicleManager.cpp (110): Could not reach provided broker\n";
        return false;
    }
}

bool VehicleManager::create_tcp_client(const std::string& client_id,
    const std::string& ip_addr_device) {
    if ( !check_ip_addr_validity(ip_addr_device) ) {
        std::cerr << "In VehicleManager.cpp (102): Device IP address format is not valid\n";
        return false;
    }
    // else return its_status_msg.connect_flag;
    else return true;
    // its_client.reset(new tod_network::MqttClientTemplated<VehicleManager>(ip_addr_device, client_id));
    // if (its_client->is_connected()) {
    //     return true;
    // } else {
    //     std::cerr << "In VehicleManager.cpp (110): Could not reach provided broker\n";
    //     return false;
    // }
}

void VehicleManager::create_and_run_ros_thread() {
        its_thread = std::thread(&rosloop_vehicle::ros_run_spin_loop,
                             &its_ros_vehicle_loop, "VehicleManagerNode", "status_msg",
                             &its_status_msg, &ros_terminated);
    
}

void VehicleManager::create_close_qt_thread() {
    its_close_qt_thread = std::thread(&VehicleManager::wait_to_shut_down_qt_loop, this);
}

void VehicleManager::wait_to_shut_down_qt_loop() {
    while ( ros::ok() ) {
    }
    wait_for_ros_thread_to_join();
    its_v2xManagerWindow->quitAll();
}

void VehicleManager::shutdown_qt_widget() {
    its_v2xManagerWindow->quitAll();
}

void VehicleManager::wait_for_ros_thread_to_join() {
    its_thread.join();
    printf("ros thread joined\n");
}

void VehicleManager::set_mqtt_pub_topic_to(const std::string& topic) {
    its_mqtt_status_msg_topic = topic;
}

void VehicleManager::pub_its_status_msg_to_broker() {
    update_vehicle_time_stamp();
    its_mutex.lock();
    ros::SerializedMessage rosSer = ros::serialization::serializeMessage
        <tod_msgs::Status>(its_status_msg);
    its_mutex.unlock();
    its_client->publish(its_mqtt_status_msg_topic, 1,
        (char*) rosSer.message_start, rosSer.num_bytes);
}

void VehicleManager::set_mqtt_callback_to_topic(const std::string& topic_name) {
    its_client->subscribe(topic_name, 1,
        &VehicleManager::callback_await_response, this);
}

void VehicleManager::callback_await_response(const mqtt::const_message_ptr msg) {
    ros::serialization::IStream stream((uint8_t*) msg->get_payload_str().c_str(),
        msg->get_payload().length());
    tod_msgs::Status tmp;
    ros::serialization::Serializer<tod_msgs::Status>::read(stream, tmp);
    // set_tod_status_of_its_status_msg(tmp);
    set_operator_part_of_its_status_msg(tmp);
    // emit new_vehicle_timestamp((unsigned int) its_status_msg.vehicle_header.stamp.sec);
    if ( number_received_packages >= INT_MAX ) {
        number_received_packages = 0;
    }
    ++number_received_packages;
}

void VehicleManager::init_its_status_msg_data() {
    its_status_msg.tod_status = tod_msgs::Status::TOD_STATUS_IDLE;
    its_status_msg.vehicle_EPS_approved = false;
    its_status_msg.vehicle_ACC_approved = false;
    its_status_msg.vehicle_EPS_Status = false;
    its_status_msg.vehicle_ACC_Status = false;
    its_status_msg.vehicle_emergency_stop_released = false;
    its_status_msg.operator_PC_ip_address = "127.0.0.1";
    its_status_msg.connect_flag = false;
    its_status_msg.operator_control_mode =
        tod_msgs::Status::CONTROL_MODE_DIRECT;
    // its_status_msg.vehicle_control_mode =
    //     tod_msgs::Status::CONTROL_MODE_DIRECT;
    its_status_msg.operator_video_mode =
        tod_msgs::Status::VIDEO_RATE_CONTROL_MODE_SINGLE;
    its_status_msg.operator_input_device = InputDevice::G29;
    update_vehicle_time_stamp();
}  

void VehicleManager::callback_safety_driver_status(const tod_msgs::SafetyDriverStatus&
        safety_driver_status_msg) {
    tod_msgs::Status temporary_status_msg;
    temporary_status_msg.vehicle_emergency_stop_released =
        safety_driver_status_msg.vehicle_emergency_stop_released;
    temporary_status_msg.vehicle_EPS_approved =
        safety_driver_status_msg.vehicle_EPS_approved;
    temporary_status_msg.vehicle_ACC_approved =
        safety_driver_status_msg.vehicle_ACC_approved;
    temporary_status_msg.vehicle_EPS_Status =
        safety_driver_status_msg.vehicle_EPS_Status;
    temporary_status_msg.vehicle_ACC_Status =
        safety_driver_status_msg.vehicle_ACC_Status;
    set_vehicle_part_of_its_status_msg(temporary_status_msg);
}

void VehicleManager::callback_control_command_data(const tod_msgs::ControlCmd &
        control_command_msg) {
    tod_msgs::ControlCmd temporary_controlCmd_msg;
    temporary_controlCmd_msg.operator_id = control_command_msg.operator_id;
    temporary_controlCmd_msg.control_type = control_command_msg.control_type;
    temporary_controlCmd_msg.remote_flag = control_command_msg.remote_flag;
    temporary_controlCmd_msg.streaming_flag = control_command_msg.streaming_flag;
    temporary_controlCmd_msg.steeringWheelAngle = control_command_msg.steeringWheelAngle;
    temporary_controlCmd_msg.gearPosition = control_command_msg.gearPosition;
    temporary_controlCmd_msg.velocity = control_command_msg.velocity;
    temporary_controlCmd_msg.acceleration = control_command_msg.acceleration;
    temporary_controlCmd_msg.indicator = control_command_msg.indicator;
    set_operator_part_of_its_controlCmd_msg(temporary_controlCmd_msg);
}

void VehicleManager::callback_pvd_data(const tod_msgs::PVD_data &pvd_data_msg) {
    tod_msgs::PVD_data temporary_pvdData_msg;
    temporary_pvdData_msg.vehicle_name = pvd_data_msg.vehicle_name;
    temporary_pvdData_msg.vehicle_id = pvd_data_msg.vehicle_id;
    temporary_pvdData_msg.vehicle_type = pvd_data_msg.vehicle_type;
    temporary_pvdData_msg.velocity = pvd_data_msg.velocity;
    temporary_pvdData_msg.gear_status = pvd_data_msg.gear_status;
    temporary_pvdData_msg.steering_wheel = pvd_data_msg.steering_wheel;
    temporary_pvdData_msg.acceleration = pvd_data_msg.acceleration;
    temporary_pvdData_msg.latitude = pvd_data_msg.latitude;
    temporary_pvdData_msg.longitude = pvd_data_msg.longitude;
    temporary_pvdData_msg.elevation = pvd_data_msg.elevation;
    temporary_pvdData_msg.heading = pvd_data_msg.heading;
    temporary_pvdData_msg.vehicle_mode = pvd_data_msg.vehicle_mode;
    set_vehicle_part_of_its_pvd_msg(temporary_pvdData_msg);
}

void VehicleManager::set_control_mode_to_its_status_msg(const uint8_t control_mode) {
    its_mutex.lock();
    if(its_status_msg.mode_status==1)
        its_status_msg.operator_control_mode = control_mode;
    else
        its_status_msg.vehicle_control_mode = control_mode;
    its_mutex.unlock();
    update_vehicle_time_stamp();
}

void VehicleManager::set_video_rate_control_mode_to_its_status_msg(const uint8_t video_control_mode) {
    its_mutex.lock();
    its_status_msg.operator_video_mode = video_control_mode;
    its_mutex.unlock();
    update_vehicle_time_stamp();
}

void VehicleManager::set_tod_status_of_its_status_msg(const uint8_t connection_status) {
    its_mutex.lock();
    its_status_msg.tod_status = connection_status;
    its_mutex.unlock();
    update_vehicle_time_stamp();
}

void VehicleManager::set_v2x_header_of_its_status_msg(const std::vector<uint16_t> header_values) {
    its_mutex.lock();
    if(its_status_msg.mode_status==1){
        its_status_msg.oper_v2x_header = header_values;
    }
    else{
        its_status_msg.vehicle_v2x_header = header_values;
    }
    its_mutex.unlock();
    update_vehicle_time_stamp();
}

void VehicleManager::set_mode_status_of_its_status_msg(const uint8_t mode_status) {
    its_mutex.lock();
    its_status_msg.mode_status = mode_status;
    its_mutex.unlock();
    update_vehicle_time_stamp();
}

void VehicleManager::set_operator_part_of_its_status_msg(const tod_msgs::Status& msg) {
    its_mutex.lock();
    its_status_msg.operator_header = msg.operator_header;
    its_status_msg.operator_control_mode = msg.operator_control_mode;
    its_status_msg.operator_video_mode = msg.operator_video_mode;
    its_status_msg.operator_device_ip_address = msg.operator_device_ip_address;
    its_status_msg.operator_PC_ip_address = msg.operator_PC_ip_address;
    its_status_msg.operator_device_port = msg.operator_device_port;
    its_status_msg.vehicle_control_mode = msg.operator_control_mode;
    its_status_msg.oper_v2x_header = msg.oper_v2x_header;
    its_mutex.unlock();
    update_vehicle_time_stamp();
}

void VehicleManager::set_operator_part_of_its_controlCmd_msg(const tod_msgs::ControlCmd& msg) {
    its_mutex.lock();
    its_controlCmd_msg.operator_id = msg.operator_id;
    its_controlCmd_msg.control_type = msg.control_type;
    its_controlCmd_msg.remote_flag = msg.remote_flag;
    its_controlCmd_msg.streaming_flag = msg.streaming_flag;
    its_controlCmd_msg.steeringWheelAngle = msg.steeringWheelAngle;
    its_controlCmd_msg.gearPosition = msg.gearPosition;
    its_controlCmd_msg.velocity = msg.velocity;
    its_controlCmd_msg.acceleration = msg.acceleration;
    its_controlCmd_msg.indicator = msg.indicator;

    // std::cout << "steering wheel : "<<msg.steeringWheelAngle<<std::endl;
    its_mutex.unlock();
    update_vehicle_time_stamp();
}

void VehicleManager::set_vehicle_part_of_its_pvd_msg(const tod_msgs::PVD_data& msg) {
    its_mutex.lock();
    its_pvdData_msg.vehicle_name = msg.vehicle_name;
    its_pvdData_msg.vehicle_id = msg.vehicle_id;
    its_pvdData_msg.vehicle_type = msg.vehicle_type;
    its_pvdData_msg.velocity = msg.velocity;
    its_pvdData_msg.gear_status = msg.gear_status;
    its_pvdData_msg.steering_wheel = msg.steering_wheel;
    its_pvdData_msg.acceleration = msg.acceleration;
    its_pvdData_msg.latitude = msg.latitude;
    its_pvdData_msg.longitude = msg.longitude;
    its_pvdData_msg.elevation = msg.elevation;
    its_pvdData_msg.heading = msg.heading;
    its_pvdData_msg.vehicle_mode = msg.vehicle_mode;

    // std::cout << "steering wheel : "<<msg.steeringWheelAngle<<std::endl;
    its_mutex.unlock();
    update_vehicle_time_stamp();
}

void VehicleManager::set_vehicle_part_of_its_status_msg(const tod_msgs::Status& msg) {
    its_mutex.lock();
    its_status_msg.vehicle_header = msg.vehicle_header;
    // its_status_msg.vehicle_control_mode = msg.vehicle_control_mode;
    its_status_msg.vehicle_EPS_approved = msg.vehicle_EPS_approved;
    its_status_msg.vehicle_ACC_approved = msg.vehicle_ACC_approved;
    its_status_msg.vehicle_EPS_Status = msg.vehicle_EPS_Status;
    its_status_msg.vehicle_ACC_Status = msg.vehicle_ACC_Status;
    its_status_msg.vehicle_emergency_stop_released = msg.vehicle_emergency_stop_released;
    // its_status_msg.vehicle_nav_status = msg.vehicle_nav_status;
    // its_status_msg.vehicle_gps_pos_type = msg.vehicle_gps_pos_type;
    its_mutex.unlock();
    update_vehicle_time_stamp();
}


void VehicleManager::set_ip_addr_of_its_status_msg(const std::string& ip_addr_pc,
    const std::string& ip_addr_device, const int ip_device_port) {
    its_mutex.lock();
    if(gui_status.mode_status==1){
        its_status_msg.operator_PC_ip_address = ip_addr_pc;
        its_status_msg.operator_device_ip_address = ip_addr_device;
        its_status_msg.operator_device_port = ip_device_port;
    }
    else{
        its_status_msg.vehicle_PC_ip_address = ip_addr_pc;
        its_status_msg.vehicle_device_ip_address = ip_addr_device;
        its_status_msg.vehicle_device_port = ip_device_port;
    }
    its_mutex.unlock();
    update_vehicle_time_stamp();
}

void VehicleManager::start_frequent_mqtt_pub(const int frequency) {
    its_mqtt_thread = std::thread(&VehicleManager::mqtt_pub_loop, this, frequency);
}

void VehicleManager::mqtt_pub_loop(const int frequency) {
    mqtt_pub_loop_is_running = true;
    while ( !stop_mqtt_pub_loop ) {
        pub_its_status_msg_to_broker();
        // emit new_operator_timestamp((unsigned int) ros::Time::now().sec);
        usleep((unsigned int) frequency*1000);
    }
    stop_mqtt_pub_loop = false;
    mqtt_pub_loop_is_running = false;
}

void VehicleManager::stop_frequent_mqtt_pub() {
    stop_mqtt_pub_loop = true;
    its_mqtt_thread.join();
    printf("mqtt pub thread joined\n");
}

int VehicleManager::num_rec_mqtt_packages() {
    return number_received_packages;
}

bool VehicleManager::check_if_mqtt_client_is_connected() {
    if ( its_client == nullptr ) { return false; }
    return its_client->is_connected();
}

void VehicleManager::stop_mqtt_if_running() {
    if ( mqtt_pub_loop_is_running ) {
        stop_frequent_mqtt_pub();
    }
}

tod_msgs::Status VehicleManager::get_its_status_msg() {
    return its_status_msg;
}

void VehicleManager::update_vehicle_time_stamp() {
    ros::Time::init();
    its_mutex.lock();
    if(its_status_msg.mode_status==1){
        its_status_msg.operator_header.stamp = ros::Time::now();
    }
    else{
        its_status_msg.vehicle_header.stamp = ros::Time::now();
    }
    its_mutex.unlock();
}



void VehicleManager::close_application_if_needed() {
    if ( ros_terminated ) {
        ros_terminated = false;
        printf("\n");
        printf("VehicleManager: STARTED TO SHUTDOWN THE APPLICATION - WAITING FOR THREADS TO JOIN\n");
        // stop_mqtt_if_running();
        wait_for_ros_thread_to_join();
        QApplication::quit();
        printf("\n");
    }
}

void VehicleManager::stop_mqtt_if_idle() {
    if ( its_status_msg.tod_status == tod_msgs::Status::TOD_STATUS_IDLE ) {
        stop_mqtt_if_running();
    }
}



void VehicleManager::update_gui_if_different_from_its_tod_status() {
    gui_status = its_v2xManagerWindow->get_gui_status();
    if (gui_status.vehicle_emergency_stop_released != its_status_msg.vehicle_emergency_stop_released) {
        emit signal_vehicle_emergency_stop_released(its_status_msg.vehicle_emergency_stop_released);
    }
    if ( gui_status.vehicle_EPS_approved != its_status_msg.vehicle_EPS_approved ) {
        emit signal_vehicle_EPS_approved(its_status_msg.vehicle_EPS_approved);
    }
    if ( gui_status.vehicle_ACC_approved != its_status_msg.vehicle_ACC_approved ) {
        emit signal_vehicle_ACC_approved(its_status_msg.vehicle_ACC_approved);
    }
    its_status_msg.mode_status=gui_status.mode_status;
    // printf("tod status : %d\n",gui_status.tod_status);
    // std::cout<<"tod_status : "<<gui_status.operator_control_mode<<std::endl;;
    // if ( gui_status.vehicle_nav_status != its_status_msg.vehicle_nav_status ) {
    //     emit signal_update_nav_status(its_status_msg.vehicle_nav_status);
    // }
    // if ( gui_status.vehicle_gps_pos_type != its_status_msg.vehicle_gps_pos_type ) {
    //     emit signal_update_gps_pos_type(its_status_msg.vehicle_gps_pos_type);
    // }
}

void VehicleManager::update_gui_status_from_its_data() {
    // tod_msgs::ControlCmd gui_controlCmd = its_v2xManagerWindow->get_gui_controlCmd();
    // set_operator_part_of_its_controlCmd_msg(its_ros_operator_loop.controlCmdStatus);
    emit signal_pvd_data(its_pvdData_msg);
    emit signal_control_command_data(its_controlCmd_msg);
    // if(its_status_msg.mode_status==1){
    //     emit signal_control_command_data(its_controlCmd_msg);
    // }
    // else{
    //     emit signal_pvd_data(its_pvdData_msg);
    // }

}

void VehicleManager::processStudyLeaderWishes(const tod_msgs::Status studyLeaderWishes) {
    if ( its_status_msg.tod_status == tod_msgs::Status::TOD_STATUS_IDLE &&
    studyLeaderWishes.tod_status == tod_msgs::Status::TOD_STATUS_UPLINK_ONLY) {
        emit signal_Connect(studyLeaderWishes.operator_device_ip_address,
            studyLeaderWishes.operator_device_port);
    }
    if ( its_status_msg.tod_status == tod_msgs::Status::TOD_STATUS_UPLINK_ONLY &&
    studyLeaderWishes.tod_status == tod_msgs::Status::TOD_STATUS_TELEOPERATION ) {
        set_control_mode_to_its_status_msg(studyLeaderWishes.operator_control_mode);
        emit signal_Start();
    }
    if ( its_status_msg.tod_status == tod_msgs::Status::TOD_STATUS_TELEOPERATION &&
    studyLeaderWishes.tod_status == tod_msgs::Status::TOD_STATUS_UPLINK_ONLY ) {
        emit signal_Stop();
    }
    if ( studyLeaderWishes.tod_status == tod_msgs::Status::TOD_STATUS_IDLE ) {
        emit signal_Disconnect();
    }

    // emit signal_control_command_data(its_controlCmd_msg);
}
