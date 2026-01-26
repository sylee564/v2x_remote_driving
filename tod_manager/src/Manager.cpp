#include "tod_manager/Manager.h"

#include <boost/asio/ip/address_v4.hpp>
#include <QDate>
#include <QDateTime>

// ===== 생성/소멸 =====
Manager::Manager(const std::string& pathToYamlFile,
                 const std::string& searchedKey)
  : ip_addr_checker(new IPv4ValidityChecker)
{
  // ---- 초기 상태 ----
  its_status_msg.tod_status = tod_msgs::Status::TOD_STATUS_IDLE;
  its_status_msg.operator_control_mode = tod_msgs::Status::CONTROL_MODE_DIRECT;
  its_status_msg.operator_video_mode = 0;
  its_status_msg.streaming_mode = 0;
  its_status_msg.operator_v2x_connected_status = 0;

  its_status_msg.vehicle_emergency_stop_released = 0;
  its_status_msg.vehicle_lat_control_status = 0;
  its_status_msg.vehicle_lon_control_status = 0;
  its_status_msg.vehicle_streaming_status = 0;
  its_status_msg.vehicle_v2x_connected_status = 0;

  // ---- 윈도우 생성 및 연결 ----
  its_v2xManagerWindow = new v2xManagerWindow(pathToYamlFile, searchedKey, this);

  connect(its_v2xManagerWindow, &v2xManagerWindow::signal_on_pushButton_DBconfigSave_clicked,
          this, &Manager::handle_signal_on_db_save_clicked);
  connect(its_v2xManagerWindow, &v2xManagerWindow::signal_on_connectBtn_Connect_clicked,
          this, &Manager::handle_signal_on_connect_clicked);
  connect(its_v2xManagerWindow, &v2xManagerWindow::signal_on_connectBtn_Disconnect_clicked,
          this, &Manager::handle_signal_on_disconnect_clicked);
  connect(its_v2xManagerWindow, &v2xManagerWindow::signal_on_StartAndStopBtn_Start_clicked,
          this, &Manager::handle_signal_on_start_clicked);
  connect(its_v2xManagerWindow, &v2xManagerWindow::signal_on_StartAndStopBtn_Stop_clicked,
          this, &Manager::handle_signal_on_stop_clicked);
  connect(its_v2xManagerWindow, &v2xManagerWindow::signal_on_pushButton_PSID_Add_Delete_clicked,
          this, &Manager::handle_signal_on_psid_add_delete_clicked);
  connect(this, &Manager::signal_psid_register_result,
        its_v2xManagerWindow, &v2xManagerWindow::handle_psid_apply_result);        
  connect(its_v2xManagerWindow, &v2xManagerWindow::signal_input_device_changed,
          this, &Manager::call_input_device_change_service);
  connect(its_v2xManagerWindow, &v2xManagerWindow::signal_save_directory_path_changed,
          this, &Manager::handle_signal_on_save_directory_path);
  connect(its_v2xManagerWindow, &v2xManagerWindow::signal_on_radioBtn_Select_clicked,
          this, &Manager::on_mode_changed);
  connect(its_v2xManagerWindow, &v2xManagerWindow::control_value_changed,
          this, &Manager::handle_signal_control_sens_value);

  connect(this, &Manager::signal_control_command_data,
          its_v2xManagerWindow, &v2xManagerWindow::get_control_command_data);
  connect(this, &Manager::signal_probe_vehicle_data,
          its_v2xManagerWindow, &v2xManagerWindow::get_probe_vehicle_data);
  connect(this, &Manager::signal_v2x_stats_info,
          its_v2xManagerWindow, &v2xManagerWindow::get_v2x_stats);
  connect(this, &Manager::signal_modem_status,
          its_v2xManagerWindow, &v2xManagerWindow::get_modem_status);
  connect(this, &Manager::signal_com_status,
          its_v2xManagerWindow, &v2xManagerWindow::get_com_status);
  connect(this, &Manager::signal_network_status,
          its_v2xManagerWindow, &v2xManagerWindow::change_network_status);
  connect(this, &Manager::signal_socket_id,
          its_v2xManagerWindow, &v2xManagerWindow::get_socket_id);

  connect(this, &Manager::signal_vehicle_emergency_stop_released,
          its_v2xManagerWindow, &v2xManagerWindow::change_emergency_stop_released);
  connect(this, &Manager::signal_vehicle_lat_approved,
          its_v2xManagerWindow, &v2xManagerWindow::lat_approved);
  connect(this, &Manager::signal_vehicle_lon_approved,
          its_v2xManagerWindow, &v2xManagerWindow::lon_approved);

    {
    ros::NodeHandle pnh("~");
    bool tmp{};
    if (pnh.getParam("isVehicle", tmp)) {
      isVehicle_ = tmp;
    } else {
      nh_.param<bool>("isVehicle", isVehicle_, false);
    }
  }

  // GUI 초기 모드 동기화 (파라미터 반영)
  if (its_v2xManagerWindow) its_v2xManagerWindow->setInitialMode(isVehicle_);

  // ---- ROS 구독 ----
  sub_tod_status_  = nh_.subscribe<tod_msgs::Status>("sub_tod_status", 10, &Manager::callback_tod_status, this);
  sub_ctrl_cmd_    = nh_.subscribe<tod_msgs::ControlCmd>("/tod/control_cmd", 10, &Manager::callback_control_command_data, this);
  sub_pvd_         = nh_.subscribe<tod_msgs::ProbeVehicleData>("/tod/probe_vehicle_data", 10, &Manager::callback_probe_vehicle_data, this);
  sub_v2xstat_     = nh_.subscribe<nr_v2x_msgs::V2XStat>("/nr_v2x/stats", 10, &Manager::callback_v2x_stats, this);
  sub_modem_rx_    = nh_.subscribe<nr_v2x_msgs::ModemRxStatus>("/nr_v2x/modem_rx", 10, &Manager::callback_modem_rx, this);
  sub_modem_tx_    = nh_.subscribe<nr_v2x_msgs::ModemTxStatus>("/nr_v2x/modem_tx", 10, &Manager::callback_modem_tx, this);
  sub_com_status_  = nh_.subscribe<nr_v2x_msgs::CommUnitStatus>("/nr_v2x/comm_status", 10, &Manager::callback_com_status, this);
  sub_network_     = nh_.subscribe<tcpip_msgs::status>("/tcpip/status", 10, &Manager::callback_network_status, this);

  wsr_client_ = nh_.serviceClient<nr_v2x_msgs::Wsr>("wsr_tx");
  // ros::service::waitForService("wsr_tx", ros::Duration(2.0));

  if (isVehicle_) {
    sub_acc_status_   = nh_.subscribe<kona_driver_msgs::AccReport>("acc_report", 10, &Manager::callback_lon_status, this);
    sub_steer_status_ = nh_.subscribe<kona_driver_msgs::SteerReport>("steer_report", 10, &Manager::callback_lat_status, this);
  }

  // ---- /tod/status 퍼블리셔 + 퍼블리싱 스레드 (20Hz) ----
  pub_tod_status_ = nh_.advertise<tod_msgs::Status>("pub_tod_status", 10, false);

  status_pub_running_ = true;
  status_pub_thread_ = std::thread([this](){
    ros::Rate r(20.0);
    while (ros::ok() && status_pub_running_) {
      tod_msgs::Status out;
      {
        std::lock_guard<std::mutex> lk(mtx_status); // 원본을 수정하므로 반드시 락
        const ros::Time now = ros::Time::now();
        if (isVehicle_) {
          its_status_msg.vehicle_header.seq++;
          its_status_msg.vehicle_header.stamp = now;
        } else {
          its_status_msg.operator_header.seq++;
          its_status_msg.operator_header.stamp = now;
        }
        out = its_status_msg; // 증가 반영된 상태로 복사
      }

      pub_tod_status_.publish(out);
      r.sleep();
    }
  });

  // ---- GUI 타이머 ----
  check_timer_100 = new QTimer(this);
  check_timer_10  = new QTimer(this);
  connect(check_timer_10,  &QTimer::timeout, this, &Manager::update_gui_if_different_from_its_tod_status);
  connect(check_timer_100, &QTimer::timeout, this, &Manager::update_gui_v2x_stats_info);
  connect(check_timer_10,  &QTimer::timeout, this, &Manager::update_gui_status_from_its_data);
  connect(check_timer_10,  &QTimer::timeout, this, &Manager::close_application_if_needed);
  check_timer_10->start(10);
  check_timer_100->start(100);
}

Manager::~Manager() {
  // 퍼블리시 스레드 정리
  status_pub_running_ = false;
  if (status_pub_thread_.joinable()) status_pub_thread_.join();

  if (its_v2xManagerWindow) delete its_v2xManagerWindow;
  if (check_timer_10)  { check_timer_10->stop();  delete check_timer_10;  }
  if (check_timer_100) { check_timer_100->stop(); delete check_timer_100; }
}

void Manager::show_window() {
  if (its_v2xManagerWindow) its_v2xManagerWindow->show();
}

// (선택) 예전 rosloop_manager 사용 분기 – 현재 미사용
void Manager::create_and_run_ros_thread() {}
void Manager::wait_for_ros_thread_to_join() {}
void Manager::create_close_qt_thread() {}
void Manager::wait_to_shut_down_qt_loop() {
  while (ros::ok()) {}
  shutdown_qt_window();
}
void Manager::shutdown_qt_window() { if (its_v2xManagerWindow) its_v2xManagerWindow->quitAll(); }

// ===== DB 저장 =====
void Manager::start_db_save(const int frequency) {
  data_save_thread = std::thread(&Manager::db_save_loop, this, frequency);
}
void Manager::stop_db_save() {
  stop_db_save_loop = true;
  if (file.is_open())  file.close();
  if (s_file.is_open()) s_file.close();
  if (data_save_thread.joinable()) data_save_thread.join();
  stop_db_save_loop = false;
}
void Manager::db_save_loop(const int frequency) {
  db_loop_is_running = true;
  QTime time; time.start();
  while (!stop_db_save_loop) {
    const int elapsed = time.elapsed();
    if (elapsed > 60000) { // 1분마다 롤오버
      if (file.is_open())  file.close();
      if (s_file.is_open()) s_file.close();
      if (!current_dir.empty()) create_csvfile(current_dir);
      time.restart();
    }
    usleep((unsigned int)frequency * 1000); // ms→us
  }
  db_loop_is_running = false;
}

std::string Manager::create_directory(const std::string path) {
  QDate date_now = QDate::currentDate();
  std::string folder_path = path + "/" + date_now.toString(Qt::ISODate).toStdString();
  boost::filesystem::path p(folder_path);
  if (!boost::filesystem::exists(p)) boost::filesystem::create_directories(folder_path);

  boost::filesystem::create_directories(folder_path + "/sender");
  boost::filesystem::create_directories(folder_path + "/receive");
  return folder_path;
}
void Manager::create_csvfile(const std::string dirPath) {
  QDateTime time_now = QDateTime::currentDateTime();
  std::string r = dirPath + "/receive/" + time_now.toString("yyyy-MM-dd hh:mm").toStdString() + ".csv";
  std::string s = dirPath + "/sender/"  + time_now.toString("yyyy-MM-dd hh:mm").toStdString() + ".csv";
  if (!boost::filesystem::exists(r)) file.open(r);
  if (!boost::filesystem::exists(s)) s_file.open(s);
}

// ===== ROS Callbacks =====
void Manager::callback_tod_status(const tod_msgs::StatusConstPtr& msg) {
  if (!msg) return;
  set_its_status_msg(*msg);
  send_status = true;
}

void Manager::callback_control_command_data(const tod_msgs::ControlCmdConstPtr& msg) {
  if (!msg) return;
  if (msg->aeb_flag && isVehicle_)
    its_status_msg.vehicle_emergency_stop_released = 1;
  else
    its_status_msg.vehicle_emergency_stop_released = 0;

  set_its_controlCmd_msg(*msg);
  send_status = true;
}

void Manager::callback_probe_vehicle_data(const tod_msgs::ProbeVehicleDataConstPtr& msg) {
  if (!msg) return;
  set_its_pvd_msg(*msg);
}

void Manager::callback_v2x_stats(const nr_v2x_msgs::V2XStatConstPtr& msg) {
  if (!msg) return;
  set_its_v2x_stat(*msg);

  // its_v2x_stat_ = *msg;
}

void Manager::callback_modem_rx(const nr_v2x_msgs::ModemRxStatusConstPtr& msg) {
  if (!msg) return;
  last_modem_rx_ = *msg; have_rx_ = true;
  if (have_rx_ && have_tx_) emit signal_modem_status(last_modem_rx_, last_modem_tx_);
}

void Manager::callback_modem_tx(const nr_v2x_msgs::ModemTxStatusConstPtr& msg) {
  if (!msg) return;
  last_modem_tx_ = *msg; have_tx_ = true;
  if (have_rx_ && have_tx_) emit signal_modem_status(last_modem_rx_, last_modem_tx_);
}

void Manager::callback_com_status(const nr_v2x_msgs::CommUnitStatusConstPtr& msg) {
  if (!msg) return;
  emit signal_com_status(*msg);
}

void Manager::callback_network_status(const tcpip_msgs::statusConstPtr& msg) {
  if (!msg) return;
  set_its_network_msg(*msg);
}

void Manager::callback_lat_status(const kona_driver_msgs::SteerReportConstPtr& msg) {
  if (!msg) return;
  its_status_msg.vehicle_lat_control_status = msg->eps_control_status;
}

void Manager::callback_lon_status(const kona_driver_msgs::AccReportConstPtr& msg) {
  if (!msg) return;
  its_status_msg.vehicle_lon_control_status = msg->acc_control_status;
}

// ===== 서비스/핸들러 =====
void Manager::handle_signal_on_db_save_clicked(const nr_v2x_msgs::V2XDataConfig v2x_db){
  set_v2x_db_of_its_status_msg(v2x_db);
}

void Manager::handle_signal_on_connect_clicked(const std::string& ip_addr, const int port, const std::string& ip_addr_dev) {
  const bool isOperator = (mode_status_ == 1);
  if (!ip_addr_checker->validate(ip_addr)) {
    std::cerr << "Manager: IP address format is not valid\n";
    return;
  }

  if (!ip_addr_checker->validate(ip_addr_dev)) {
    std::cerr << "Manager: Device IP address format is not valid\n";
    return;
  }

  uint32_t socket_id = 0;
  bool success = false;

  boost::asio::ip::address addr     = boost::asio::ip::address::from_string(ip_addr);
  boost::asio::ip::address dev_addr = boost::asio::ip::address::from_string(ip_addr_dev);
  std::string s_port = std::to_string(port);

  if (!isOperator) { // Vehicle(Client)
    tcpip_msgs::start_tcp_client req;
    std::memcpy(req.request.local_endpoint.ip.data(),  addr.to_v4().to_bytes().data(), 4);
    std::memcpy(req.request.remote_endpoint.ip.data(), dev_addr.to_v4().to_bytes().data(), 4);
    req.request.remote_endpoint.port = DEST_PORT;
    success   = ros::service::call("/start_tcp_client", req);
    socket_id = req.response.client_id;
    if (success) {
      m_client.insert({socket_id, ip_addr});
      set_tod_status_of_its_status_msg(tod_msgs::Status::TOD_STATUS_UPLINK_ONLY);
    }
  } else {          // Operator(Server)
    tcpip_msgs::start_tcp_server req;
    std::memcpy(req.request.local_endpoint.ip.data(), addr.to_v4().to_bytes().data(), 4);
    req.request.local_endpoint.port = port;
    success   = ros::service::call("/start_tcp_server", req);
    socket_id = req.response.server_id;
    if (success) {
      m_server.insert({socket_id, ip_addr + ":" + s_port});
      set_tod_status_of_its_status_msg(tod_msgs::Status::TOD_STATUS_UPLINK_ONLY);
    }
  }
  update_gui_socket_id(success, socket_id);
}

void Manager::handle_signal_on_disconnect_clicked(uint32_t socket_id) {
  bool success{false};

  if (its_status_msg.operator_control_mode == tod_msgs::Status::CONTROL_MODE_DIRECT) {
    tcpip_msgs::stop_tcp_client stop_client;
    stop_client.request.client_id = socket_id;
    success = ros::service::call("/stop_tcp_client", stop_client);
    its_close_socket.request.socket_id = socket_id;
    success = ros::service::call("/close_socket", its_close_socket);
    if (success) {
      m_client.erase(socket_id);
      if (m_client.empty())
        set_tod_status_of_its_status_msg(tod_msgs::Status::TOD_STATUS_IDLE);
    }
  } else {
    tcpip_msgs::stop_tcp_server stop_server;
    stop_server.request.server_id = socket_id;
    success = ros::service::call("/stop_tcp_server", stop_server);
    its_close_socket.request.socket_id = socket_id;
    success = ros::service::call("/close_socket", its_close_socket);
    if (success) {
      m_server.erase(socket_id);
      if (m_server.empty())
        set_tod_status_of_its_status_msg(tod_msgs::Status::TOD_STATUS_IDLE);
    }
  }
}

void Manager::handle_signal_on_start_clicked() {
  set_tod_status_of_its_status_msg(tod_msgs::Status::TOD_STATUS_TELEOPERATION);
  if (!current_dir.empty()) { create_csvfile(current_dir); start_db_save(20); }
}

void Manager::handle_signal_on_stop_clicked() {
  set_tod_status_of_its_status_msg(tod_msgs::Status::TOD_STATUS_UPLINK_ONLY);
  if (db_loop_is_running) stop_db_save();
}

void Manager::handle_signal_on_psid_add_delete_clicked(uint32_t psid, int action) {
  std::thread([this, psid, action]() {
    nr_v2x_msgs::Wsr srv;
    srv.request.action       = static_cast<uint8_t>(action); // 0:Add, 1:Delete
    srv.request.request_psid = psid;

    bool ok = wsr_client_.call(srv);
    bool success = ok && srv.response.success;

    // 서버가 액션/PSID를 변환해서 돌려줄 수도 있으면 그 값을 사용
    const int      final_action = ok ? static_cast<int>(srv.response.action_result) : action;
    const uint32_t final_psid   = ok ? srv.response.response_psid : psid;

    // 반드시 GUI 스레드에서 emit
    QMetaObject::invokeMethod(
      this,
      [this, success, final_action, final_psid]() {
        emit signal_psid_register_result(success, final_action, final_psid);
      },
      Qt::QueuedConnection
    );
  }).detach();

}

void Manager::call_input_device_change_service(const std::string& input_device) {
  tod_msgs::InputDevice input_device_request;
  input_device_request.request.input_device_directory = input_device;
  bool success = ros::service::call("change_input_device", input_device_request);
  if (!success) {
    ROS_ERROR("Could not reach service server for input device change at %s",
              ros::this_node::getName().c_str());
  }
}

void Manager::handle_signal_on_save_directory_path(const std::string cur_directory) {
  current_dir = create_directory(cur_directory);
}

void Manager::on_mode_changed(uint8_t mode) {
  mode_status_ = mode; // 0: Vehicle, 1: Operator
}

void Manager::handle_signal_control_sens_value(const std::string& /*name*/, int /*value*/) {
  // 필요 시 파라미터화
}

// ===== 내부 상태 갱신 =====
void Manager::set_tod_status_of_its_status_msg(uint8_t s) {
  std::lock_guard<std::mutex> lk(mtx_status);
  its_status_msg.tod_status = s;
  update_time_stamp();
}

void Manager::set_v2x_db_of_its_status_msg(const nr_v2x_msgs::V2XDataConfig& cfg)
{
  const char* kSrvName = "/v2x_config";
  if (!ros::service::exists(kSrvName, /*print_failure=*/false)) {
    ROS_ERROR_STREAM("Manager: Service " << kSrvName << " is not available.");
    return;
  }

  nr_v2x_msgs::V2XConfig srv;
  srv.request.psid_ext   = cfg.psid_ext;
  srv.request.cast_mode  = cfg.cast_mode;
  srv.request.src_id     = cfg.src_id;
  srv.request.dst_id     = cfg.dst_id;

  srv.request.eDeviceType   = cfg.eDeviceType;
  srv.request.eTeleCommType = cfg.eTeleCommType;
  srv.request.unDeviceId    = cfg.unDeviceId;
  srv.request.eServiceId    = cfg.eServiceId;
  srv.request.eActionType   = cfg.eActionType;
  srv.request.eRegionId     = cfg.eRegionId;
  srv.request.ePayloadType  = cfg.ePayloadType;
  srv.request.eCommId       = cfg.eCommId;
  srv.request.usDbVer       = cfg.usDbVer;
  srv.request.usHwVer       = cfg.usHwVer;
  srv.request.usSwVer       = cfg.usSwVer;

  const bool ok = ros::service::call(kSrvName, srv);
  if (!ok) {
    ROS_ERROR_STREAM("Manager: Service call to " << kSrvName << " failed (transport error).");
    return;
  }
  if (srv.response.success) {
    ROS_INFO_STREAM("Manager: V2X config applied. msg=\"" << srv.response.message << "\"");
  } else {
    ROS_WARN_STREAM("Manager: V2X config rejected by server. msg=\"" << srv.response.message << "\"");
  }
}

void Manager::set_its_status_msg(const tod_msgs::Status& msg) {
  std::lock_guard<std::mutex> lk(mtx_status);
  if (isVehicle_) {
    its_status_msg.operator_control_mode = msg.operator_control_mode;
    its_status_msg.operator_video_mode   = msg.operator_video_mode;
    its_status_msg.streaming_mode        = msg.streaming_mode;
    its_status_msg.operator_v2x_connected_status = msg.operator_v2x_connected_status;
    its_status_msg.tod_status            = msg.tod_status;
  } else {
    its_status_msg.vehicle_emergency_stop_released = msg.vehicle_emergency_stop_released;
    its_status_msg.vehicle_lat_control_status      = msg.vehicle_lat_control_status;
    its_status_msg.vehicle_lon_control_status      = msg.vehicle_lon_control_status;
    its_status_msg.vehicle_streaming_status        = msg.vehicle_streaming_status;
    its_status_msg.vehicle_v2x_connected_status    = msg.vehicle_v2x_connected_status;
  }
}

void Manager::set_its_controlCmd_msg(const tod_msgs::ControlCmd& msg) {
  std::lock_guard<std::mutex> lk(mtx_cmd);
  its_controlCmd_msg = msg;
  update_time_stamp();
}

void Manager::set_its_pvd_msg(const tod_msgs::ProbeVehicleData& msg) {
  std::lock_guard<std::mutex> lk(mtx_pvd);
  its_pvd_msg = msg;
  update_time_stamp();
}

void Manager::set_its_v2x_stat(const nr_v2x_msgs::V2XStat& msg) {
  std::lock_guard<std::mutex> lk(mtx_v2x);
  its_v2x_stat_ = msg;
  // emit signal_v2x_stats_info(its_v2x_stat_);

}

void Manager::set_its_network_msg(const tcpip_msgs::status& msg) {
  std::lock_guard<std::mutex> lk(mtx_net);
  its_network_msg = msg;
}

void Manager::set_control_mode_to_its_status_msg(uint8_t control_mode) {
  std::lock_guard<std::mutex> lk(mtx_status);
  its_status_msg.operator_control_mode = control_mode;
  update_time_stamp();
}

void Manager::set_video_rate_control_mode_to_its_status_msg(uint8_t operator_video_mode) {
  std::lock_guard<std::mutex> lk(mtx_status);
  its_status_msg.operator_video_mode = operator_video_mode;
  update_time_stamp();
}

void Manager::update_time_stamp() {
  ros::Time::init();
  if (!isVehicle_) {
    its_status_msg.operator_header.stamp = ros::Time::now();
  } else {
    its_status_msg.vehicle_header.stamp = ros::Time::now();
  }
}

// ===== 주기적 GUI 갱신 =====
void Manager::close_application_if_needed() {
  if (ros_terminated) {
    ros_terminated = false;
    QApplication::quit();
  }
}

void Manager::update_gui_if_different_from_its_tod_status() {
  gui_status  = its_v2xManagerWindow->get_gui_status();
  gui_network = its_v2xManagerWindow->get_network_status();

  if (gui_status.vehicle_emergency_stop_released != its_status_msg.vehicle_emergency_stop_released)
    emit signal_vehicle_emergency_stop_released(its_status_msg.vehicle_emergency_stop_released);
  if (gui_status.vehicle_lat_control_status != its_status_msg.vehicle_lat_control_status)
    emit signal_vehicle_lat_approved(its_status_msg.vehicle_lat_control_status);
  if (gui_status.vehicle_lon_control_status != its_status_msg.vehicle_lon_control_status)
    emit signal_vehicle_lon_approved(its_status_msg.vehicle_lon_control_status);

  bool any_added = false;

  for (const auto& s : its_network_msg.tcp_sockets) {
    if (!seen_socket_ids_.insert(s.id).second) continue;  // 이미 본 ID는 스킵


    // 원격 주소:포트 추출
    boost::asio::ip::address_v4::bytes_type b;
    std::memcpy(b.data(), s.remote_endpoint.ip.data(), 4);
    boost::asio::ip::address_v4 remote_addr(b);
    const std::string remote_port = std::to_string(s.remote_endpoint.port);
    
    if (mode_status_ == 0) {
      m_server.insert({s.id, remote_addr.to_string() + ":" + remote_port});
    } else {
      m_client.insert({s.id, remote_addr.to_string() + ":" + remote_port});
    }


    emit signal_network_status(its_network_msg);
    any_added = true;
  }
  // emit signal_network_status(its_network_msg, "", /*socket_add_flag*/ false);
}

void Manager::update_gui_status_from_its_data() {
  emit signal_probe_vehicle_data(its_pvd_msg);
  emit signal_control_command_data(its_controlCmd_msg);

}

void Manager::update_gui_v2x_stats_info() {
  emit signal_v2x_stats_info(its_v2x_stat_);
}

// ===== GUI 보조 =====
void Manager::update_gui_socket_id(bool success, uint32_t socket_id) {
  emit signal_socket_id(success, socket_id);
}