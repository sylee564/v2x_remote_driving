#pragma once

// ===== C++ / STL =====
#include <memory>
#include <mutex>
#include <deque>
#include <vector>
#include <map>
#include <thread>
#include <atomic>
#include <fstream>
#include <cstdint>
#include <unordered_set>

// ===== POSIX =====
#include <unistd.h>   // usleep

// ===== Qt =====
#include <QObject>
#include <QTime>
#include <QTimer>
#include <QApplication>

// ===== ROS =====
#include <ros/ros.h>

// ===== Boost =====
#include <boost/asio/ip/address.hpp>
#include <boost/filesystem.hpp>

// ===== MSGs =====
#include "tod_msgs/Status.h"
#include "tod_msgs/ControlCmd.h"
#include "tod_msgs/ProbeVehicleData.h"
#include "tod_msgs/inputDevice.h"
#include "tod_msgs/InputDevice.h"

#include "nr_v2x_msgs/V2XDataConfig.h"
#include "nr_v2x_msgs/V2XConfig.h"
#include "nr_v2x_msgs/V2XStat.h"
#include "nr_v2x_msgs/ModemRxStatus.h"
#include "nr_v2x_msgs/ModemTxStatus.h"
#include "nr_v2x_msgs/CommUnitStatus.h"
#include "nr_v2x_msgs/Wsr.h"

#include "kona_driver_msgs/SteerReport.h"
#include "kona_driver_msgs/AccReport.h"

#include <tcpip_msgs/start_tcp_client.h>
#include <tcpip_msgs/stop_tcp_client.h>
#include <tcpip_msgs/start_tcp_server.h>
#include <tcpip_msgs/stop_tcp_server.h>
#include <tcpip_msgs/close_socket.h>
#include <tcpip_msgs/status.h>

// ===== Local =====
#include "IpAddrValidityChecker.h"
#include "IPv4ValidityChecker.h"
#include "v2xManagerWindow.h"

#define DEST_PORT 47347

class Manager : public QObject {
  Q_OBJECT
public:
  Manager(const std::string& pathToYamlFile, const std::string& searchedKey);
  ~Manager();

  void show_window();
  void create_and_run_ros_thread();        // (선택) 사용 안 함: 퍼블리시 스레드만 사용 중
  void wait_for_ros_thread_to_join();      // (선택) 사용 안 함
  void create_close_qt_thread();           // (선택) 사용 안 함
  void shutdown_qt_window();
  void wait_to_shut_down_qt_loop();

  // 서비스/핸들러
  void call_input_device_change_service(const std::string& input_device);
  void handle_signal_on_save_directory_path(const std::string cur_directory);
  void handle_signal_on_connect_clicked(const std::string& ip_addr, const int port, const std::string& ip_addr_dev);
  void handle_signal_on_disconnect_clicked(uint32_t socket_id);
  void handle_signal_on_start_clicked();
  void handle_signal_on_stop_clicked();
  void handle_signal_on_psid_add_delete_clicked(uint32_t psid, int action);
  void handle_signal_on_db_save_clicked(const nr_v2x_msgs::V2XDataConfig v2x_db);
  void handle_signal_control_sens_value(const std::string& name, int value);

  // DB 저장 쓰레드
  void start_db_save(const int frequency);
  void stop_db_save();

signals:
  void signal_vehicle_emergency_stop_released(uint8_t released);
  void signal_vehicle_lat_approved(uint8_t approved);
  void signal_vehicle_lon_approved(uint8_t approved);
  void signal_control_command_data(tod_msgs::ControlCmd);
  void signal_probe_vehicle_data(tod_msgs::ProbeVehicleData);
  void signal_v2x_stats_info(nr_v2x_msgs::V2XStat);
  void signal_modem_status(nr_v2x_msgs::ModemRxStatus, nr_v2x_msgs::ModemTxStatus);
  void signal_com_status(nr_v2x_msgs::CommUnitStatus);
  void signal_network_status(tcpip_msgs::status);
  void signal_socket_id(bool success, uint32_t socket_id);
  void signal_psid_register_result(bool success, int action, uint32_t psid);
  void signal_Disconnect();

private:
  // 콜백
  void callback_tod_status(const tod_msgs::StatusConstPtr& msg);
  void callback_control_command_data(const tod_msgs::ControlCmdConstPtr& msg);
  void callback_probe_vehicle_data(const tod_msgs::ProbeVehicleDataConstPtr& msg);
  void callback_v2x_stats(const nr_v2x_msgs::V2XStatConstPtr& msg);
  void callback_modem_rx(const nr_v2x_msgs::ModemRxStatusConstPtr& msg);
  void callback_modem_tx(const nr_v2x_msgs::ModemTxStatusConstPtr& msg);
  void callback_com_status(const nr_v2x_msgs::CommUnitStatusConstPtr& msg);
  void callback_network_status(const tcpip_msgs::statusConstPtr& msg);
  void callback_lat_status(const kona_driver_msgs::SteerReportConstPtr& msg);
  void callback_lon_status(const kona_driver_msgs::AccReportConstPtr& msg);

  // 내부 상태 갱신
  void on_mode_changed(uint8_t mode);
  void set_tod_status_of_its_status_msg(uint8_t connection_status);
  void set_v2x_db_of_its_status_msg(const nr_v2x_msgs::V2XDataConfig& cfg);
  void set_its_status_msg(const tod_msgs::Status& msg);
  void set_its_controlCmd_msg(const tod_msgs::ControlCmd& msg);
  void set_its_pvd_msg(const tod_msgs::ProbeVehicleData& msg);
  void set_its_v2x_stat(const nr_v2x_msgs::V2XStat& msg);
  void set_its_network_msg(const tcpip_msgs::status& msg);
  void set_control_mode_to_its_status_msg(uint8_t control_mode);
  void set_video_rate_control_mode_to_its_status_msg(uint8_t operator_video_mode);
  void update_time_stamp();

  // 주기적 GUI 갱신
  void close_application_if_needed();
  void update_gui_if_different_from_its_tod_status();
  void update_gui_status_from_its_data();
  void update_gui_v2x_stats_info();
  void update_gui_socket_id(bool success, uint32_t socket_id);

  // DB
  void db_save_loop(const int frequency);
  std::string create_directory(const std::string path);
  void create_csvfile(const std::string dirPath);

private:
  // 유틸
  std::unique_ptr<IpAddrValidityChecker> ip_addr_checker;
  std::thread its_thread, its_close_qt_thread, data_save_thread;

  // ===== ROS pub/sub =====
  ros::NodeHandle nh_;
  ros::Subscriber sub_ctrl_cmd_, sub_pvd_, sub_v2xstat_, sub_modem_rx_, sub_modem_tx_, sub_com_status_, sub_network_;
  ros::Subscriber sub_tod_status_;
  ros::Subscriber sub_acc_status_, sub_steer_status_;
  ros::ServiceClient wsr_client_;
  // 상태 발행자와 발행 스레드 (20Hz)
  ros::Publisher pub_tod_status_;
  std::thread status_pub_thread_;
  std::atomic<bool> status_pub_running_{false};

  // GUI/데이터
  v2xManagerWindow* its_v2xManagerWindow{nullptr};
  std::mutex mtx_status;   // its_status_msg
  std::mutex mtx_pvd;      // its_pvd_msg
  std::mutex mtx_cmd;      // its_controlCmd_msg
  std::mutex mtx_net;      // its_network_msg
  std::mutex mtx_v2x;      // its_v2x_stat_
  tod_msgs::Status its_status_msg{}, gui_status{};
  tod_msgs::ControlCmd its_controlCmd_msg{};
  tod_msgs::ProbeVehicleData its_pvd_msg{};
  nr_v2x_msgs::V2XStat its_v2x_stat_{};
  nr_v2x_msgs::ModemRxStatus last_modem_rx_{};
  nr_v2x_msgs::ModemTxStatus last_modem_tx_{};
  bool have_rx_{false}, have_tx_{false};

  tcpip_msgs::status its_network_msg{}, gui_network{};
  bool isVehicle_{false};

  // 타이머(메인 스레드에서만 구동)
  QTimer* check_timer_100{nullptr};
  QTimer* check_timer_10{nullptr};

  // 네트워크/저장
  std::unordered_set<uint32_t> seen_socket_ids_;
  std::map<uint32_t, std::string> m_server;
  std::map<uint32_t, std::string> m_client;
  std::string current_dir;
  std::ofstream file, s_file;
  bool stop_db_save_loop{false}, db_loop_is_running{false};
  bool recv_status{false}, send_status{false};

  tcpip_msgs::close_socket its_close_socket;
  bool ros_terminated{false};

  // 모드 캐시(소켓 추가 방향 판단에 사용)
  uint8_t mode_status_{0};
};