#ifndef V2XMANAGERWINDOW_H
#define V2XMANAGERWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QVector>
#include <QString>
#include <QtWebChannel/QWebChannel>
#include <QWebEngineView>
#include <QWebEngineProfile>
#include <QWebEngineSettings>
#include <QUrl>
#include <QDateTime>
#include <QFileDialog>
#include <QHostAddress>
#include <QSharedPointer>
#include <QVBoxLayout>
#include <QFile>
#include <QDir>
#include <QRegExp>
#include <QMessageBox>

#include <map>
#include <vector>
#include <string>
#include <cstdint>
#include <optional>
#include <cmath>
#include <algorithm>
#include <boost/asio/ip/address.hpp>

#include "QtWidgetGroup.h"
#include "map_bridge.hpp"
#include "qcustomplot.h"
#include "MapCanvasGL.h"

#include "QRateLimiter.hpp"
#include "CoalescedTimer.hpp"

// ROS/msgs
#include <ros/package.h>
#include "tod_msgs/Status.h"
#include "tod_msgs/ControlCmd.h"
#include "tod_msgs/ProbeVehicleData.h"
#include "tod_msgs/VehicleEnums.h"

#include "nr_v2x_msgs/V2XDataBase.h"
#include "nr_v2x_msgs/V2XDataConfig.h"
#include "nr_v2x_msgs/V2XStat.h"
#include "nr_v2x_msgs/CommUnitStatus.h"
#include "nr_v2x_msgs/ModemRxStatus.h"
#include "nr_v2x_msgs/ModemTxStatus.h"

#include <tcpip_msgs/status.h>

QT_BEGIN_NAMESPACE
namespace Ui { class v2xManagerWindow; }
QT_END_NAMESPACE

// IP 우선 정렬(구현은 cpp)
bool sortIpAddresses(const std::string& a, const std::string& b);

class QCPGraph;
class MapCanvasGL;

class v2xManagerWindow : public QMainWindow {
  Q_OBJECT
public:
  explicit v2xManagerWindow(const std::string& pathToYamlFile,
                            const std::string& searchedKey,
                            QObject* operatorManager,
                            QWidget* parent = nullptr);
  ~v2xManagerWindow();

  tod_msgs::Status   get_gui_status()     { return gui_status; }
  tcpip_msgs::status get_network_status() { return gui_network_msg; }
  void set_gui_connection_status_to(uint8_t s) { gui_status.tod_status = s; }
  void set_gui_status_control_mode(uint8_t m)  { gui_status.operator_control_mode = m; }

  // 초기 모드를 파라미터에 맞춰 UI와 내부 상태에 반영
  Q_INVOKABLE void setInitialMode(bool isVehicle);


signals:
  void signal_on_connectBtn_Connect_clicked(const std::string& ip_addr,
                                            int port,
                                            const std::string& ip_addr_dev);
  void signal_on_connectBtn_Disconnect_clicked(uint32_t socket_id);
  void signal_on_StartAndStopBtn_Start_clicked();
  void signal_on_StartAndStopBtn_Stop_clicked();
  void signal_on_pushButton_PSID_Add_Delete_clicked(uint32_t psid, int action);
  void signal_control_mode_changed(uint8_t control_mode);
  void signal_input_device_changed(const std::string& input_device);
  void signal_video_mode_changed(uint8_t video_mode);
  void signal_on_pushButton_DBconfigSave_clicked(const nr_v2x_msgs::V2XDataConfig V2XConfig);
  void signal_save_directory_path_changed(const std::string cur_directory);
  void signal_on_radioBtn_Select_clicked(uint8_t mode_status);
  void control_value_changed(const std::string &name, int value); 

public slots:
  // 종료
  void quitAll();

  // 좌측 메뉴
  void on_statusBtn_clicked();
  void on_mapBtn_clicked();
  void on_statusV2XBtn_clicked();
  void on_dataBtn_clicked();
  void on_reportBtn_clicked();
  void on_settingBtn_clicked();
  void on_infoBtn_clicked();

  // 라디오버튼
  void on_radioButton_Vehicle_clicked();
  void on_radioButton_Operator_clicked();

  // 연결/시작/정지
  void on_connectBtn_clicked();
  void on_StartAndStopBtn_clicked();

  // 리스트/PSID 조작
  void on_pushButton_ServerDel_clicked();
  void on_pushButton_ClientDel_clicked();
  void on_pushButton_PSID_Add_clicked();
  void on_pushButton_PSID_Delete_clicked();

  // map 파일
  void on_BrowseAndOkBtn_map_clicked();

  // 파일 선택/DB 저장
  void on_BrowseAndOkBtn_clicked();
  void on_SaveDirectoryBtn_clicked();
  void on_pushButton_DBconfigSave_clicked();

  // 모드/비디오 모드
  void on_PushButton_DirectControl_clicked();
  void on_PushButton_IndirectControl_clicked();
  void on_PushButton_VideoMode_Single_clicked();
  void on_PushButton_VideoMode_Multiple_clicked();

  // 외부 입력으로 상태 갱신
  void change_emergency_stop_released(uint8_t released);
  void lat_approved(uint8_t approved);
  void lon_approved(uint8_t approved);
  void handle_psid_apply_result(bool ok, int action, uint32_t psid);
  void get_socket_id(bool success, uint32_t socket_id);

  // 데이터 수신 → GUI 반영
  void get_control_command_data(const tod_msgs::ControlCmd& control_data);
  void get_probe_vehicle_data(const tod_msgs::ProbeVehicleData& pvd_msg);
  void get_v2x_stats(const nr_v2x_msgs::V2XStat& v2x_stats_msg);
  void get_modem_status(const nr_v2x_msgs::ModemRxStatus& modem_rx,
                        const nr_v2x_msgs::ModemTxStatus& modem_tx);
  void get_com_status(const nr_v2x_msgs::CommUnitStatus& com_status);

  // 네트워크 상태
  void change_network_status(const tcpip_msgs::status& network_msg);

  // 차트/맵
  void init_map(); // (지연 로드 정책으로 내부에서 호출 안 함 — 필요 시 외부에서 수동 호출)
  void refresh_graph();
  void add_point(QCPGraph* g, double t, double v);
  void open_lanelet2_map();

private:
  // Connect 버튼 입력값 검증 (IP 두 개 + Port)
  bool validateConnectInputs(QString ip, QString portStr, QString devIp, QString* err) const;

private:
  // 내부 초기화
  void init_gui_status();
  void init_plot();
  void register_control_mode_buttons();
  void register_video_mode_buttons();

  bool loadLanelet2IntoCanvas(const QString& file);
  void setup_lanelet2_page();

  void bind_slider_and_lineEdit(QSlider *slider, QLineEdit *lineEdit, const std::string &name);

  // 네트워크 보조
  void readAndStoreOwnIpAddresses();
  void addIpAddressesToComboBox();
  void change_button_status_after_clicked_on_connect(bool conFlag);
  void change_ui_status_after_clicked_mode(bool mode_flag);
  void update_safety_driver_status_labels();

  // socket_id로 LinkMetrics 찾기
  const tcpip_msgs::LinkMetrics* findMetricsBySocketId(const tcpip_msgs::status& st,
                                                       uint32_t socketId) const;

  // 웹엔진/지도 최적화
  void setup_webview_settings_();
  void setWebMapEnabled_(bool on);

  // 헤더 채우기
  void fillComboBox(std::vector<std::string> configNodes);
  void fill_list_widget(std::string configNode);
  void setVehicleComboBox();
  void setOperatorComboBox();
  void update_header();

private:
  // Connect 버튼 입력값 검증 (IP 두 개 + Port)
  

private:
  // UI
  Ui::v2xManagerWindow* ui{nullptr};
  QCustomPlot* plot_ = nullptr;

  // 경로/모드
  std::string _pathToYamlFile;
  uint8_t     mode_status{0};

  // IP/포트/헤더
  std::vector<std::string> _listOfIpAddressesPC;
  std::vector<std::string> ipAddresses;
  std::vector<std::string> portNumbers;

  // 콤보 데이터
  std::map<uint16_t, std::string> deviceTypes;
  std::map<uint16_t, std::string> serviceIDs;
  std::map<uint16_t, std::string> telecomTypes;
  std::map<uint16_t, std::string> regionIDs;
  std::map<uint16_t, std::string> actionTypes;
  std::map<uint16_t, std::string> comIDs;
  std::map<uint16_t, std::string> payloadTypes;
  std::map<uint16_t, std::string> psids;

  // 상태
  nr_v2x_msgs::V2XDataConfig gui_v2x_db{};
  tod_msgs::Status           gui_status{};
  tcpip_msgs::status         gui_network_msg{};

  // 버튼 그룹
  QtWidgetGroup control_buttons;
  QtWidgetGroup video_mode_buttons;

  // Plot_Data 시리즈 메타
  struct MetricSeries { QCPGraph* graph{nullptr}; QString yLabel; };
  std::map<QString, MetricSeries> metrics_;
  QString   current_metric_;
  QTimer*   graph_timer_{nullptr};
  qint64    start_time_ms_{0};
  double    seconds_show_on_graph{120.0};

  // 맵/웹 채널
  QWebChannel* web_channel_{nullptr};
  MapBridge*   map_bridge_{nullptr};
  MapCanvasGL* lanelet_canvas_{nullptr};

  // WebEngine 최적화/토글
  bool   web_map_loaded_  = false;   // 처음엔 로드 안 함(지연 로드)
  bool   web_map_enabled_ = false;   // Lanelet2와 동시 렌더 방지 토글

  // WebChannel 전송 레이트 제한 + 변화량 임계치
  QRateLimiter carpos_rl_{100};      // 10 Hz로 웹맵 전송
  double last_web_lon_ = 0.0, last_web_lat_ = 0.0, last_web_heading_ = 0.0;
  const  double kMinDeltaMeter_   = 0.5; // 0.5m 이상 이동시만 웹맵 갱신
  const  double kMinDeltaHeading_ = 2.0; // 2도 이상 회전 시만

  // Lanelet2 캔버스 업데이트 스케줄러(최대 20Hz)
  std::unique_ptr<CoalescedTimer> laneletFlush_;

  // QCustomPlot 데이터 상한 및 샘플링
  int    max_points_per_series_ = 1200;   // 60초@20Hz
  bool   adaptive_sampling_on_  = true;

  // 네트워크 뷰용 마지막 소켓
  uint32_t last_added_socket_id_ = 0;
  QSet<uint32_t> listed_socket_ids_;
};

#endif // V2XMANAGERWINDOW_H