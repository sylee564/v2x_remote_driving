#include "tod_manager/v2xManagerWindow.h"
#include "ui_v2xManagerWindow.h"
#include "tod_manager/v2xHeaderDeserializer.h"

#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

// ----- 우선 정렬 우선순위 -----
static const char* PRIO_LOOPBACK = "127.0.0.";
static const char* PRIO_LAN      = "192.168.";

// ----- V2X DB 고정값(예시) -----
#define DeviceId 0x00000001
#define DbVer    0x0001
#define HwVer    0x0001
#define SwVer    0x0001

// YAML 키
static const std::string IpAddressKey       = "IpAddress";
static const std::string PortKey            = "Port";
static const std::string DeviceTypeKey      = "DeviceType";
static const std::string ServiceIDKey       = "ServiceID";
static const std::string TelecomTypeKey     = "TelecomType";
static const std::string RegionIDKey        = "RegionID";
static const std::string ActionTypeKey      = "ActionType";
static const std::string CommunicationIDKey = "CommunicationID";
static const std::string PayloadTypeKey     = "PayloadType";
static const std::string PsidKey            = "PSID";

// 색상
static const QString backgroundGreen("background-color:rgb(154,205,50)");
static const QString backgroundRed  ("background-color:rgb(205,92,92)");

// 우선순위 리스트
static std::vector<std::string> g_listWithPriorities;

// 정렬 비교자
bool sortIpAddresses(const std::string& a, const std::string& b) {
  for (const auto& prio : g_listWithPriorities) {
    const bool a_has = a.find(prio) != std::string::npos;
    const bool b_has = b.find(prio) != std::string::npos;
    if (a_has && !b_has) return true;
    if (!a_has && b_has) return false;
  }
  return a < b;
}

template <typename MapT>
static QString firstValueOrEmpty(const MapT& m) {
  if (m.empty()) return {};
  return QString::fromStdString(std::begin(m)->second);
}

static QStringList toQStringList(const std::vector<std::string>& v) {
  QStringList out;
  out.reserve(static_cast<int>(v.size()));
  for (const auto& s : v) out << QString::fromStdString(s);
  return out;
}

// Port가 int일 수도 있으므로 문자열 변환 보강용
static QStringList toQStringListPorts(const std::vector<std::string>& v) {
  auto list = toQStringList(v);
  if (list.isEmpty()) {
    // 안전 기본값 (원하면 제거 가능)
    list << "47347";
  }
  return list;
}

// ---------- ctor ----------
v2xManagerWindow::v2xManagerWindow(const std::string& pathToYamlFile,
                                   const std::string& searchedKey,
                                   QObject* /*operatorManager*/,
                                   QWidget *parent)
  : QMainWindow(parent),
    ui(new Ui::v2xManagerWindow),
    _pathToYamlFile(pathToYamlFile)
{
  ui->setupUi(this);

  init_plot();
  init_gui_status();
  register_control_mode_buttons();
  register_video_mode_buttons();

  // YAML 로드
  const auto configNodes = v2xHeaderDeserializer::vectorLoad(_pathToYamlFile, searchedKey);
  fillComboBox(configNodes);
  fill_list_widget("PSID");

  // IP 콤보
  readAndStoreOwnIpAddresses();
  addIpAddressesToComboBox();

  // 초기 GUI
  ui->BrowseAndOkBtn->setEnabled(false);
  ui->radioButton_Vehicle->setChecked(true);
  on_radioButton_Vehicle_clicked();

  ui->verticalSlider_MaxVelocity->setValue(20);
  ui->verticalSlider_ControlDecel->setValue(50);
  ui->verticalSlider_ControlLateral->setValue(50);
  ui->verticalSlider_Deadzone->setValue(50);
  ui->lineEdit_MaxVelocity->setText(QString::number(ui->verticalSlider_MaxVelocity->value()));
  ui->lineEdit_ControlAccel->setText(QString::number(ui->verticalSlider_ControlAccel->value()));
  ui->lineEdit_ControlDecel->setText(QString::number(ui->verticalSlider_ControlDecel->value()));
  ui->lineEdit_ControlLateral->setText(QString::number(ui->verticalSlider_ControlLateral->value()));
  ui->lineEdit_Deadzone->setText(QString::number(ui->verticalSlider_Deadzone->value()));

  ui->Label_EmergencyBreakReleased->setStyleSheet(backgroundRed);
  ui->Label_ACCReleased->setStyleSheet(backgroundRed);
  ui->Label_EPSReleased->setStyleSheet(backgroundRed);

  bind_slider_and_lineEdit(ui->verticalSlider_MaxVelocity, ui->lineEdit_MaxVelocity, "MaxVelocity");
  bind_slider_and_lineEdit(ui->verticalSlider_ControlAccel, ui->lineEdit_ControlAccel, "ControlAccel");
  bind_slider_and_lineEdit(ui->verticalSlider_ControlDecel, ui->lineEdit_ControlDecel, "ControlDecel");
  bind_slider_and_lineEdit(ui->verticalSlider_ControlLateral, ui->lineEdit_ControlLateral, "ControlLateral");
  bind_slider_and_lineEdit(ui->verticalSlider_Deadzone, ui->lineEdit_Deadzone, "Daedzone");

  // -------- 지도 초기화(최적화) --------
  setup_lanelet2_page();   // Lanelet2 GL만 붙여둠
  setWebMapEnabled_(true); // WebEngine 지도는 지연 로드/비활성

  // Lanelet2 캔버스 업데이트(20Hz 상한) — PVD 수신에서 poke()
  laneletFlush_ = std::make_unique<CoalescedTimer>(50 /*ms*/, [this]{
    if (lanelet_canvas_) lanelet_canvas_->update();
  }, this);

  // 메트릭 콤보(중복 추가 방지)
  if (ui->ComboBox_Metric) {
    if (ui->ComboBox_Metric->findText("car_speed") < 0)
      ui->ComboBox_Metric->addItems({"car_speed","steering","latency","PDR"});
    connect(ui->ComboBox_Metric, &QComboBox::currentTextChanged, this,
      [this](const QString& k){
        for (auto& kv : metrics_) kv.second.graph->setVisible(false);
        current_metric_ = k;
        auto &s = metrics_[k];
        s.graph->setVisible(true);
        ui->StatsPlot->yAxis->setLabel(s.yLabel);
      });
  }
}

v2xManagerWindow::~v2xManagerWindow() {
  if (ros::isStarted()) ros::shutdown();
  delete ui;
  system("rosnode kill -a");
  system("killall -9 rosmaster && killall -9 rosout");
  exit(0);
}

// ---------- 내부 상태 ----------
void v2xManagerWindow::init_gui_status() {
  gui_status.operator_control_mode = tod_msgs::Status::CONTROL_MODE_DIRECT;
  gui_status.operator_video_mode = 0;
  gui_status.streaming_mode = 0;
  gui_status.operator_v2x_connected_status = 0;
  gui_status.tod_status = tod_msgs::Status::TOD_STATUS_IDLE;

  gui_status.vehicle_emergency_stop_released = 0;
  gui_status.vehicle_lat_control_status = 0;
  gui_status.vehicle_lon_control_status = 0;
  gui_status.vehicle_streaming_status = 0;
  gui_status.vehicle_v2x_connected_status = 0;
}

// ---------- Plot ----------
void v2xManagerWindow::init_plot()
{
  auto* plot = ui->StatsPlot;
  if (!plot) {
    qWarning() << "StatsPlot not found in UI";
    return;
  }

  // 제목/축 설정
  plot->plotLayout()->insertRow(0);
  plot->plotLayout()->addElement(
      0, 0, new QCPTextElement(plot, "Real-Time Plot", QFont("sans", 12, QFont::Bold)));

  auto dt = QSharedPointer<QCPAxisTickerDateTime>::create();
  dt->setDateTimeFormat("hh:mm:ss");
  plot->xAxis->setTicker(dt);
  plot->xAxis->setLabel("Time");

  plot->setNoAntialiasingOnDrag(true);
  plot->setPlottingHint(QCP::phFastPolylines, true);

  // 시리즈 생성 도우미
  auto makeSeries = [&](const QString& key, const QString& yLabel) {
    QCPGraph* g = plot->addGraph();
    g->setAntialiased(false);
    if (adaptive_sampling_on_) g->setAdaptiveSampling(true);
    g->setVisible(false);
    MetricSeries s; s.graph = g; s.yLabel = yLabel;
    metrics_[key] = s;
  };

  makeSeries("car_speed", "Velocity (m/s)");
  makeSeries("steering",  "Steering (deg)");
  makeSeries("latency",   "Latency (ms)");
  makeSeries("PDR",       "PDR (%)");

  current_metric_ = "latency";
  if (metrics_[current_metric_].graph)
    metrics_[current_metric_].graph->setVisible(true);
  plot->yAxis->setLabel(metrics_[current_metric_].yLabel);

  // 타이머 시작
  start_time_ms_ = QDateTime::currentMSecsSinceEpoch();
  if (!graph_timer_) graph_timer_ = new QTimer(this);
  connect(graph_timer_, &QTimer::timeout, this, &v2xManagerWindow::refresh_graph, Qt::UniqueConnection);
  graph_timer_->start(100); // 10 Hz
}

void v2xManagerWindow::setup_lanelet2_page() {
  QWidget* host = ui->mapWidget;
  if (!host) {
    qWarning("setup_lanelet2_page: ui->mapWidget is null");
    return;
  }

  QVBoxLayout* lay = qobject_cast<QVBoxLayout*>(host->layout());
  if (!lay) {
    lay = new QVBoxLayout(host);
    lay->setContentsMargins(0,0,0,0);
    lay->setSpacing(0);
    host->setLayout(lay);
  } else {
    while (QLayoutItem* it = lay->takeAt(0)) {
      if (auto* w = it->widget()) w->deleteLater();
      delete it;
    }
  }

  lanelet_canvas_ = new MapCanvasGL(host);
  lay->addWidget(lanelet_canvas_);

  lanelet_canvas_->setFootprintMode(MapCanvasGL::FootprintMode::Rect);
  lanelet_canvas_->setVehicleRect(4.73, 1.89, 1.0); // L, W, RearOverhang (m)

  const QString osm = QString::fromStdString(
      ros::package::getPath("tod_manager") + "/maps/lanelet2_map.osm");

  lanelet::Origin origin(lanelet::GPSPoint{36.7278803, 127.4430473, 0.0});
  lanelet_canvas_->loadLanelet2(osm, /*geoRef*/"", origin);

  lanelet_canvas_->setVehiclePoseGeo(36.7278803, 127.4430473, 0.0);
  lanelet_canvas_->centerOnVehicle();
}

void v2xManagerWindow::add_point(QCPGraph* g, double t, double v)
{
  if (!g) return;
  g->addData(t, v);
  g->data()->removeBefore(t - seconds_show_on_graph);

  // 점수 상한(발행 시간 튐 방지)
  if (g->dataCount() > max_points_per_series_) {
    const int overflow = g->dataCount() - max_points_per_series_;
    auto it = g->data()->begin();
    for (int i=0; i<overflow && it!=g->data()->end(); ++i, ++it) {
      g->data()->remove(it->key);
    }
  }
}

void v2xManagerWindow::refresh_graph()
{
  if (!ui || !ui->StatsPlot) return;

  auto it = metrics_.find(current_metric_);
  if (it == metrics_.end() || !it->second.graph) return;

  QCPGraph* g = it->second.graph;
  if (g->dataCount() == 0) return;

  const double tmax = g->data()->constEnd()[-1].key;
  const double tmin = std::max(0.0, tmax - seconds_show_on_graph);

  ui->StatsPlot->xAxis->setRange(tmin, tmax);

  static qint64 lastScaleMs = 0;
  const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
  if (nowMs - lastScaleMs > 1000) { // 1s로 완화
    ui->StatsPlot->yAxis->rescale(true);
    auto r = ui->StatsPlot->yAxis->range();
    ui->StatsPlot->yAxis->setRange(r.lower - 0.1*r.size(), r.upper + 0.1*r.size());
    lastScaleMs = nowMs;
  }

  ui->StatsPlot->replot(QCustomPlot::rpQueuedReplot);
}

// ---------- WebEngine 최적화 ----------
void v2xManagerWindow::setup_webview_settings_() {
  if (!ui || !ui->mapView) return;
  auto* page = ui->mapView->page();
  auto* settings = page->settings();

  settings->setAttribute(QWebEngineSettings::PluginsEnabled, false);
  settings->setAttribute(QWebEngineSettings::JavascriptCanOpenWindows, false);
  settings->setAttribute(QWebEngineSettings::JavascriptCanAccessClipboard, false);
  settings->setAttribute(QWebEngineSettings::AutoLoadIconsForPage, false);
  settings->setAttribute(QWebEngineSettings::Accelerated2dCanvasEnabled, false);
  settings->setAttribute(QWebEngineSettings::ScrollAnimatorEnabled, false);
  settings->setAttribute(QWebEngineSettings::LocalContentCanAccessRemoteUrls, false);
  settings->setAttribute(QWebEngineSettings::ErrorPageEnabled, false);
  settings->setAttribute(QWebEngineSettings::FullScreenSupportEnabled, false);
  // settings->setAttribute(QWebEngineSettings::WebGLEnabled, false); // 필요 시 OFF

  page->setBackgroundColor(Qt::black);

  auto* prof = QWebEngineProfile::defaultProfile();
  prof->setHttpCacheType(QWebEngineProfile::DiskHttpCache);
  prof->setPersistentCookiesPolicy(QWebEngineProfile::ForcePersistentCookies);

  page->setAudioMuted(true);
}

void v2xManagerWindow::setWebMapEnabled_(bool on) {
  if (!ui || !ui->mapView) return;
  web_map_enabled_ = on;
  if (on) {
    if (!web_map_loaded_) {
      setup_webview_settings_();
      web_channel_ = new QWebChannel(this);
      map_bridge_  = new MapBridge(this);
      web_channel_->registerObject(QStringLiteral("Bridge"), map_bridge_);
      ui->mapView->page()->setWebChannel(web_channel_);
      ui->mapView->load(QUrl(QStringLiteral("qrc:/web/map.html")));
      connect(ui->mapView, &QWebEngineView::loadFinished, this, [this](bool ok){
        if (ok && map_bridge_) emit map_bridge_->panTo(128.3985328, 35.6481281, 20.0);
      });
      web_map_loaded_ = true;
    } else {
      if (ui->mapView->url().toString() == "about:blank")
        ui->mapView->load(QUrl(QStringLiteral("qrc:/web/map.html")));
    }
    ui->mapView->setVisible(true);
  } else {
    ui->mapView->setVisible(false);
    ui->mapView->setUrl(QUrl("about:blank")); // 렌더/JS 정지
  }
}

// ---------- 버튼 그룹 등록 ----------
void v2xManagerWindow::register_control_mode_buttons() {
  control_buttons.addWidget(ui->PushButton_DirectControl);
  control_buttons.addWidget(ui->PushButton_IndirectControl);
}
void v2xManagerWindow::register_video_mode_buttons() {
  video_mode_buttons.addWidget(ui->PushButton_VideoMode_Single);
  video_mode_buttons.addWidget(ui->PushButton_VideoMode_Multiple);
}

// ---------- 콤보/리스트 ----------
void v2xManagerWindow::fillComboBox(std::vector<std::string> configNodes)
{
  for (const auto& node : configNodes) {
    if (node == "IpAddress") {
      auto ips = v2xHeaderDeserializer::vectorLoad(_pathToYamlFile, node);
      for (const auto& s : toQStringList(ips))
        ui->ComboBox_IpAddressDevice->addItem(s);

    } else if (node == "Port") {
      auto ports = v2xHeaderDeserializer::vectorLoad(_pathToYamlFile, node);
      for (const auto& s : toQStringListPorts(ports))
        ui->ComboBox_Port->addItem(s);

    } else if (node == "DeviceType") {
      deviceTypes = v2xHeaderDeserializer::mapLoad(_pathToYamlFile, node);
      for (const auto& kv : deviceTypes)
        ui->ComboBox_DeviceType->addItem(QString::fromStdString(kv.second));

    } else if (node == "ServiceID") {
      serviceIDs = v2xHeaderDeserializer::mapLoad(_pathToYamlFile, node);
      for (const auto& kv : serviceIDs)
        ui->ComboBox_ServiceID->addItem(QString::fromStdString(kv.second));

    } else if (node == "TelecomType") {
      telecomTypes = v2xHeaderDeserializer::mapLoad(_pathToYamlFile, node);
      for (const auto& kv : telecomTypes)
        ui->ComboBox_TelecomType->addItem(QString::fromStdString(kv.second));

    } else if (node == "RegionID") {
      regionIDs = v2xHeaderDeserializer::mapLoad(_pathToYamlFile, node);
      for (const auto& kv : regionIDs)
        ui->ComboBox_RegionID->addItem(QString::fromStdString(kv.second));

    } else if (node == "ActionType") {
      actionTypes = v2xHeaderDeserializer::mapLoad(_pathToYamlFile, node);
      for (const auto& kv : actionTypes)
        ui->ComboBox_ActionType->addItem(QString::fromStdString(kv.second));

    } else if (node == "CommunicationID") {
      comIDs = v2xHeaderDeserializer::mapLoad(_pathToYamlFile, node);
      for (const auto& kv : comIDs)
        ui->ComboBox_ComID->addItem(QString::fromStdString(kv.second));

    } else if (node == "PayloadType") {
      payloadTypes = v2xHeaderDeserializer::mapLoad(_pathToYamlFile, node);
      for (const auto& kv : payloadTypes)
        ui->ComboBox_PayloadType->addItem(QString::fromStdString(kv.second));
    }
  }

  if (ui->ComboBox_IpAddressDevice->count() == 0)
    ui->ComboBox_IpAddressDevice->addItem("127.0.0.1");
  if (ui->ComboBox_Port->count() == 0)
    ui->ComboBox_Port->addItem("47347");
}

void v2xManagerWindow::fill_list_widget(std::string configNode)
{
  psids = v2xHeaderDeserializer::mapLoad(_pathToYamlFile, configNode);
  ui->listWidget_PSID_Left->clear();
  if (psids.empty())
    return;

  for (const auto& kv : psids)
    ui->listWidget_PSID_Left->addItem(QString::fromStdString(kv.second));
}

// ---------- 기본 콤보 세트 ----------
void v2xManagerWindow::setVehicleComboBox()
{
  if (!deviceTypes.empty())
    ui->ComboBox_DeviceType->setCurrentText(firstValueOrEmpty(deviceTypes));
  if (!serviceIDs.empty())
    ui->ComboBox_ServiceID->setCurrentText(firstValueOrEmpty(serviceIDs));
  if (!telecomTypes.empty())
    ui->ComboBox_TelecomType->setCurrentText(firstValueOrEmpty(telecomTypes));
  if (!actionTypes.empty())
    ui->ComboBox_ActionType->setCurrentText(firstValueOrEmpty(actionTypes));
  if (!regionIDs.empty())
    ui->ComboBox_RegionID->setCurrentText(firstValueOrEmpty(regionIDs));
  if (!comIDs.empty())
    ui->ComboBox_ComID->setCurrentText(firstValueOrEmpty(comIDs));
  if (!payloadTypes.empty())
    ui->ComboBox_PayloadType->setCurrentText(firstValueOrEmpty(payloadTypes));
}
void v2xManagerWindow::setOperatorComboBox() { setVehicleComboBox(); }

// ---------- 헤더 구성 ----------
void v2xManagerWindow::update_header() {
  for (const auto& kv : deviceTypes)
    if (kv.second == ui->ComboBox_DeviceType->currentText().toStdString())
      gui_v2x_db.eDeviceType = kv.first;
  for (const auto& kv : telecomTypes)
    if (kv.second == ui->ComboBox_TelecomType->currentText().toStdString())
      gui_v2x_db.eTeleCommType = kv.first;

  gui_v2x_db.unDeviceId = DeviceId;

  for (const auto& kv : serviceIDs)
    if (kv.second == ui->ComboBox_ServiceID->currentText().toStdString())
      gui_v2x_db.eServiceId = kv.first;
  for (const auto& kv : actionTypes)
    if (kv.second == ui->ComboBox_ActionType->currentText().toStdString())
      gui_v2x_db.eActionType = kv.first;
  for (const auto& kv : regionIDs)
    if (kv.second == ui->ComboBox_RegionID->currentText().toStdString())
      gui_v2x_db.eRegionId = kv.first;
  for (const auto& kv : payloadTypes)
    if (kv.second == ui->ComboBox_PayloadType->currentText().toStdString())
      gui_v2x_db.ePayloadType = kv.first;
  for (const auto& kv : comIDs)
    if (kv.second == ui->ComboBox_ComID->currentText().toStdString())
      gui_v2x_db.eCommId = kv.first;

  gui_v2x_db.usDbVer = DbVer;
  gui_v2x_db.usHwVer = HwVer;
  gui_v2x_db.usSwVer = SwVer;
}

// ---------- IP ----------
void v2xManagerWindow::readAndStoreOwnIpAddresses() {
  struct ifaddrs* ifAddrStruct = nullptr;
  if (getifaddrs(&ifAddrStruct) == -1) return;

  for (auto* ifa = ifAddrStruct; ifa != nullptr; ifa = ifa->ifa_next) {
    if (!ifa->ifa_addr) continue;
    if (ifa->ifa_addr->sa_family == AF_INET) {
      auto* sa = (struct sockaddr_in*)(ifa->ifa_addr);
      char addr[INET_ADDRSTRLEN] = {0};
      inet_ntop(AF_INET, &(sa->sin_addr), addr, INET_ADDRSTRLEN);
      _listOfIpAddressesPC.emplace_back(addr);
    }
  }
  if (ifAddrStruct) freeifaddrs(ifAddrStruct);

  g_listWithPriorities = { std::string(PRIO_LAN), std::string(PRIO_LOOPBACK) };
  std::sort(_listOfIpAddressesPC.begin(), _listOfIpAddressesPC.end(), sortIpAddresses);
}
void v2xManagerWindow::addIpAddressesToComboBox() {
  for (const auto& ip : _listOfIpAddressesPC)
    ui->ComboBox_IpAddress->addItem(QString::fromLocal8Bit(ip.c_str()));
}

// ---------- UI enable ----------
void v2xManagerWindow::change_button_status_after_clicked_on_connect(bool conFlag) {
  ui->ComboBox_IpAddress->setEnabled(!conFlag);
  ui->ComboBox_Port->setEnabled(!conFlag);
  ui->ComboBox_DeviceType->setEnabled(!conFlag);
  ui->ComboBox_ServiceID->setEnabled(!conFlag);
  ui->ComboBox_TelecomType->setEnabled(!conFlag);
  ui->ComboBox_RegionID->setEnabled(!conFlag);
  ui->ComboBox_ActionType->setEnabled(!conFlag);
  ui->ComboBox_ComID->setEnabled(!conFlag);
  ui->ComboBox_PayloadType->setEnabled(!conFlag);

  ui->radioButton_Operator->setEnabled(!conFlag);
  ui->radioButton_Vehicle->setEnabled(!conFlag);

  control_buttons.enableButtons(conFlag);
  video_mode_buttons.enableButtons(conFlag);
}

void v2xManagerWindow::change_ui_status_after_clicked_mode(bool mode_flag) {
  if (mode_flag) {
    mode_status = 1; // Operator
    setOperatorComboBox();
  } else {
    mode_status = 0; // Vehicle
    setVehicleComboBox();
    control_buttons.initialButtons();
    video_mode_buttons.initialButtons();
  }

  ui->BrowseAndOkBtn->setEnabled(mode_flag);
  ui->LineEdit_PathToInputDevice->setEnabled(mode_flag);
  ui->ComboBox_IpAddressDevice->setEnabled(!mode_flag);
  ui->ComboBox_Port->setEnabled(mode_flag);
  control_buttons.enableButtons(mode_flag);
  video_mode_buttons.enableButtons(mode_flag);
  ui->pushButton_ServerDel->setEnabled(mode_flag);

  emit signal_on_radioBtn_Select_clicked(mode_status);
}

// ---------- 라벨 ----------
void v2xManagerWindow::update_safety_driver_status_labels() {
  ui->Label_EmergencyBreakReleased->setStyleSheet(
      gui_status.vehicle_emergency_stop_released ? backgroundGreen : backgroundRed);
  ui->Label_EPSReleased->setStyleSheet(
      gui_status.vehicle_lat_control_status ? backgroundGreen : backgroundRed);
  ui->Label_ACCReleased->setStyleSheet(
      gui_status.vehicle_lon_control_status ? backgroundGreen : backgroundRed);
}

// ---------- 좌측 메뉴 ----------
void v2xManagerWindow::on_statusBtn_clicked()    { 
  ui->stackedWidget->setCurrentIndex(0);
  ui->menuLabel->setText("Status"); 
  setWebMapEnabled_(true);
}
void v2xManagerWindow::on_mapBtn_clicked() {
  ui->stackedWidget->setCurrentIndex(1);
  ui->menuLabel->setText("Lanelet2 Map");
  // Lanelet2만 활성, WebEngine 지도는 비활성
  setWebMapEnabled_(false);
  if (lanelet_canvas_) lanelet_canvas_->setVisible(true);
}
void v2xManagerWindow::on_statusV2XBtn_clicked() { ui->stackedWidget->setCurrentIndex(2); ui->menuLabel->setText("V2X Status"); }
void v2xManagerWindow::on_dataBtn_clicked()      { ui->stackedWidget->setCurrentIndex(3); ui->menuLabel->setText("Data Analysis"); }
void v2xManagerWindow::on_reportBtn_clicked()    { ui->stackedWidget->setCurrentIndex(4); ui->menuLabel->setText("Reports"); }
void v2xManagerWindow::on_settingBtn_clicked()   { ui->stackedWidget->setCurrentIndex(5); ui->menuLabel->setText("Setting"); }
void v2xManagerWindow::on_infoBtn_clicked()      { ui->stackedWidget->setCurrentIndex(6); ui->menuLabel->setText("Information"); }


// ---------- 초기 모드 동기화 (ROS 파라미터 등에서 전달받은 값을 UI/내부 상태에 반영) ----------
void v2xManagerWindow::setInitialMode(bool isVehicle)
{
  // Vehicle=true -> Vehicle 라디오 선택, Operator=false -> Operator 라디오 선택
  if (isVehicle) {
    if (!ui->radioButton_Vehicle->isChecked()) {
      ui->radioButton_Vehicle->setChecked(true);
    }
    // 내부 상태 및 버튼 활성/비활성을 모드에 맞추기
    change_ui_status_after_clicked_mode(false); // false => Vehicle 모드
  } else {
    if (!ui->radioButton_Operator->isChecked()) {
      ui->radioButton_Operator->setChecked(true);
    }
    change_ui_status_after_clicked_mode(true); // true => Operator 모드
  }
  // 모드 전환 시 리스트 및 캐시 초기화
  listed_socket_ids_.clear();
  ui->listWidget_Server->clear();
  ui->listWidget_Client->clear();
}
// ---------- 라디오 ----------
void v2xManagerWindow::on_radioButton_Vehicle_clicked()
{
  change_ui_status_after_clicked_mode(false);
  // 모드 전환 시 리스트와 캐시 초기화
  listed_socket_ids_.clear();
  ui->listWidget_Server->clear();
  ui->listWidget_Client->clear();
}
void v2xManagerWindow::on_radioButton_Operator_clicked()
{
  change_ui_status_after_clicked_mode(true);
  listed_socket_ids_.clear();
  ui->listWidget_Server->clear();
  ui->listWidget_Client->clear();
}


bool v2xManagerWindow::validateConnectInputs(QString ip, QString portStr, QString devIp, QString* err) const
{
  QHostAddress addr1, addr2;
  bool ok1 = addr1.setAddress(ip.trimmed());
  bool ok2 = addr2.setAddress(devIp.trimmed());
  bool okp = false;
  int port = portStr.toInt(&okp);
  if (!ok1) { if (err) *err = "Invalid IP: " + ip; return false; }
  if (!ok2) { if (err) *err = "Invalid Device IP: " + devIp; return false; }
  if (!okp || port < 1 || port > 65535) { if (err) *err = "Invalid Port: " + portStr; return false; }
  return true;
}
// ---------- Connect ----------
void v2xManagerWindow::on_connectBtn_clicked()
{
if (ui->connectBtn->text() == "Connect") {
  QString ip   = ui->ComboBox_IpAddress->currentText();
  QString port = ui->ComboBox_Port->currentText();
  QString dev  = ui->ComboBox_IpAddressDevice->currentText();
  QString err;
  if (!validateConnectInputs(ip, port, dev, &err)) {
    ui->Label_connectState->setText("Invalid input");
    QMessageBox::warning(this, "Connect", err);
    return;
  }
}


  if(ui->connectBtn->text()=="Connect"){
        ui->Label_connectState->setText("Uplink...");
        set_gui_connection_status_to(tod_msgs::Status::TOD_STATUS_UPLINK_ONLY);
        emit signal_on_connectBtn_Connect_clicked(
            ui->ComboBox_IpAddress->currentText().toStdString(),
            ui->ComboBox_Port->currentText().toInt(),
            ui->ComboBox_IpAddressDevice->currentText().toStdString()
        );
  }
}

// ---------- Start/Stop ----------
void v2xManagerWindow::on_StartAndStopBtn_clicked() {
  if (ui->StartAndStopBtn->text() == "Start") {
    ui->StartAndStopBtn->setText("Stop");
    ui->Label_connectState->setText("Teleoperation");
    control_buttons.enableButtons(false);

    set_gui_connection_status_to(tod_msgs::Status::TOD_STATUS_TELEOPERATION);
    emit signal_on_StartAndStopBtn_Start_clicked();
  } else {
    ui->StartAndStopBtn->setText("Start");
    ui->Label_connectState->setText("Uplink...");
    control_buttons.enableButtons(true);

    set_gui_connection_status_to(tod_msgs::Status::TOD_STATUS_UPLINK_ONLY);
    emit signal_on_StartAndStopBtn_Stop_clicked();
  }
}

// ---------- Disconnect ----------
static int extractIdFromItemText(const QString& text) {
  const QString idStr = text.split(" - ").value(0);
  bool ok = false; int id = idStr.toInt(&ok);
  return ok ? id : -1;
}
void v2xManagerWindow::on_pushButton_ServerDel_clicked() {
  if (auto* item = ui->listWidget_Server->currentItem()) {
    const int id = extractIdFromItemText(item->text());
    if (id >= 0) {
      emit signal_on_connectBtn_Disconnect_clicked(static_cast<uint32_t>(id));
      delete ui->listWidget_Server->takeItem(ui->listWidget_Server->currentRow());
    }
  }
}
void v2xManagerWindow::on_pushButton_ClientDel_clicked() {
  if (auto* item = ui->listWidget_Client->currentItem()) {
    const int id = extractIdFromItemText(item->text());
    if (id >= 0) {
      emit signal_on_connectBtn_Disconnect_clicked(static_cast<uint32_t>(id));
      delete ui->listWidget_Client->takeItem(ui->listWidget_Client->currentRow());
    }
  }
}

// ---------- PSID ----------
void v2xManagerWindow::on_pushButton_PSID_Add_clicked() {
  auto* cur = ui->listWidget_PSID_Left->currentItem();
  if (!cur) return;
  const QString name = cur->text();

  // psids: std::map<uint16_t, std::string>
  for (const auto& kv : psids) {
    if (QString::fromStdString(kv.second) == name) {
      emit signal_on_pushButton_PSID_Add_Delete_clicked(kv.first, /*action*/0); // 0:add
      return;
    }
  }
}

void v2xManagerWindow::on_pushButton_PSID_Delete_clicked() {
  auto* cur = ui->listWidget_PSID_Right->currentItem();
  if (!cur) return;
  const QString name = cur->text();

  for (const auto& kv : psids) {
    if (QString::fromStdString(kv.second) == name) {
      emit signal_on_pushButton_PSID_Add_Delete_clicked(kv.first, /*action*/1); // 1:del
      return;
    }
  }
}

void v2xManagerWindow::handle_psid_apply_result(bool ok, int action, uint32_t psid)
{
  if (!ok) {
    QMessageBox::warning(this, "PSID", "PSID 설정 실패");
    return;
  }

  // psid -> 표시 문자열(이름) 찾기
  QString targetName;
  if (auto it = psids.find(static_cast<uint16_t>(psid)); it != psids.end()) {
    targetName = QString::fromStdString(it->second);
  } else {
    // 혹시 모를 예외: 못 찾으면 선택 항목 기준으로 이동 시도
    if (action == 0 && ui->listWidget_PSID_Left->currentItem())
      targetName = ui->listWidget_PSID_Left->currentItem()->text();
    else if (action == 1 && ui->listWidget_PSID_Right->currentItem())
      targetName = ui->listWidget_PSID_Right->currentItem()->text();
    if (targetName.isEmpty()) return;
  }

  auto moveItemByName = [](QListWidget* from, QListWidget* to, const QString& name){
    for (int i=0; i<from->count(); ++i) {
      if (from->item(i)->text() == name) {
        to->addItem(from->item(i)->text());
        delete from->takeItem(i);
        return true;
      }
    }
    return false;
  };

  if (action == 0) {
    // Add: Left -> Right
    (void)moveItemByName(ui->listWidget_PSID_Left, ui->listWidget_PSID_Right, targetName);
  } else {
    // Delete: Right -> Left
    (void)moveItemByName(ui->listWidget_PSID_Right, ui->listWidget_PSID_Left, targetName);
  }
}

// ---------- map load 버튼 ----------
void v2xManagerWindow::on_BrowseAndOkBtn_map_clicked() {
  const QString file = QFileDialog::getOpenFileName(
      this, tr("Open Autoware Lanelet2 Map (.osm)"),
      QDir::homePath(),
      tr("OSM Files (*.osm *.xml)"));
  if (file.isEmpty()) return;
  ui->LineEdit_PathToMap->setText(file);
  if (!loadLanelet2IntoCanvas(file)) {
    QMessageBox::warning(this, "Error", "Failed to load Lanelet2 map");
  }
}

// ---------- DB 저장 ----------
void v2xManagerWindow::on_pushButton_DBconfigSave_clicked() {
  update_header();
  emit signal_on_pushButton_DBconfigSave_clicked(gui_v2x_db);
}

// ---------- 파일 ----------
void v2xManagerWindow::on_BrowseAndOkBtn_clicked() {
  std::string expectedPath = ros::package::getPath("tod_input_devices") + std::string("/config");
  QString fileName = QFileDialog::getOpenFileName(
      this, tr("Open Input Device Config Files"),
      QString::fromStdString(expectedPath),
      tr("Config Files (*.yaml)"));
  if (!fileName.isEmpty()) {
    ui->LineEdit_PathToInputDevice->setText(fileName);
    emit signal_input_device_changed(fileName.toStdString());
  }
}
void v2xManagerWindow::on_SaveDirectoryBtn_clicked() {
  QString directoryName = QFileDialog::getExistingDirectory(
      this, "Directory Select", QStringLiteral("/home"),
      QFileDialog::ShowDirsOnly);
  if (!directoryName.isEmpty()) {
    ui->LineEdit_PathSaveDirectory->setText(directoryName);
    emit signal_save_directory_path_changed(directoryName.toStdString());
  }
}

// ---------- 모드/비디오 ----------
void v2xManagerWindow::on_PushButton_DirectControl_clicked() {
  emit signal_control_mode_changed(tod_msgs::Status::CONTROL_MODE_DIRECT);
  control_buttons.switchFocusTo(ui->PushButton_DirectControl, backgroundGreen);
  set_gui_status_control_mode(tod_msgs::Status::CONTROL_MODE_DIRECT);
}
void v2xManagerWindow::on_PushButton_IndirectControl_clicked() {
  emit signal_control_mode_changed(tod_msgs::Status::CONTROL_MODE_INDIRECT);
  control_buttons.switchFocusTo(ui->PushButton_IndirectControl, backgroundGreen);
  set_gui_status_control_mode(tod_msgs::Status::CONTROL_MODE_INDIRECT);
}
void v2xManagerWindow::on_PushButton_VideoMode_Single_clicked()  {/* 필요 시 구현 */}
void v2xManagerWindow::on_PushButton_VideoMode_Multiple_clicked(){/* 필요 시 구현 */}

void v2xManagerWindow::bind_slider_and_lineEdit(QSlider *slider, QLineEdit *lineEdit, const std::string &name)
{
  connect(slider, &QSlider::valueChanged, this, [this, lineEdit, name](int value) {
    lineEdit->setText(QString::number(value));
    emit control_value_changed(name, value);
  });

  connect(lineEdit, &QLineEdit::editingFinished, this, [this, slider, lineEdit, name]() {
    bool ok = false;
    int value = lineEdit->text().toInt(&ok);
    if(name == "MaxVelocity"){
      if(value < 10) value = 10;
      else if(value > 50) value = 50;
    }
    if (ok) {
      slider->setValue(value);
      emit control_value_changed(name, value);
    }
  });
}

// ---------- 종료 ----------
void v2xManagerWindow::quitAll() { QApplication::quit(); }

// ---------- 상태 라벨 ----------
void v2xManagerWindow::change_emergency_stop_released(uint8_t released) {
  gui_status.vehicle_emergency_stop_released = released;
  update_safety_driver_status_labels();
}
void v2xManagerWindow::lat_approved(uint8_t approved) {
  gui_status.vehicle_lat_control_status = approved;
  update_safety_driver_status_labels();
}
void v2xManagerWindow::lon_approved(uint8_t approved) {
  gui_status.vehicle_lon_control_status = approved;
  update_safety_driver_status_labels();
}

// ---------- 소켓 ID ----------
void v2xManagerWindow::get_socket_id(bool success, uint32_t socket_id) {
  if (!success) {
    ui->Label_connectState->setText(mode_status == 0 ? "Client ip error!!" : "Server ip error!!");
    return;
  }
  const QString socket_info =
      ui->ComboBox_IpAddress->currentText() + ":" + ui->ComboBox_Port->currentText();
  const QString itemText = QString::number(socket_id) + " - " + socket_info;

  if (mode_status == 0) ui->listWidget_Client->addItem(itemText);
  else                  ui->listWidget_Server->addItem(itemText);
  last_added_socket_id_ = socket_id;
}

// ---------- 컨트롤 ----------
void v2xManagerWindow::get_control_command_data(const tod_msgs::ControlCmd& c) {
  ui->label_OperID_Value->setText(QString::fromStdString(c.operator_id));

  if      (c.control_type == tod_msgs::ControlCmd::TOD_CONTROL_RAW) ui->label_ControlType_Value->setText("RAW");
  else if (c.control_type == tod_msgs::ControlCmd::TOD_CONTROL_ACC) ui->label_ControlType_Value->setText("ACC");
  else                                                              ui->label_ControlType_Value->setText("VEL");

  ui->label_SteeringAng_Value->setText(QString::number(static_cast<int>(c.control.steering_angle)));
  switch (c.shift.data) {
    case eGearPosition::GEARPOSITION_PARK:    ui->label_OperGear_Value->setText("P"); break;
    case eGearPosition::GEARPOSITION_REVERSE: ui->label_OperGear_Value->setText("R"); break;
    case eGearPosition::GEARPOSITION_NEUTRAL: ui->label_OperGear_Value->setText("N"); break;
    case eGearPosition::GEARPOSITION_DRIVE:   ui->label_OperGear_Value->setText("D"); break;
  }

  ui->label_OperAcc_Value->setText(QString::number(c.acceleration));
  ui->label_OperThrottle_Value->setText(QString::number(c.control.throttle));
  ui->label_OperBrake_Value->setText(QString::number(c.control.brake));
  ui->label_RemoteSignal_Value->setText(c.remote_flag ? "ON" : "OFF");
  ui->label_StreamSignal_Value->setText(c.stream_flag ? "ON" : "OFF");

  switch (c.indicator.data) {
    case 3:                           ui->label_OperIndicator_Value->setText("Emergency"); break;
    case eIndicator::INDICATOR_LEFT:  ui->label_OperIndicator_Value->setText("Left"); break;
    case eIndicator::INDICATOR_RIGHT: ui->label_OperIndicator_Value->setText("Right"); break;
    default:                          ui->label_OperIndicator_Value->setText("OFF"); break;
  }
}

// ---------- 차량 데이터 ----------
void v2xManagerWindow::get_probe_vehicle_data(const tod_msgs::ProbeVehicleData& p) {
  ui->label_VehicleName_Value->setText(QString::fromStdString(p.vehicle_name));
  ui->label_VehicleID_Value->setText(QString::fromStdString(p.vehicle_id));
  ui->label_VehicleType_Value->setText(QString::number(p.vehicle_type));
  ui->label_VehicleSteering_Value->setText(QString::number(p.steering_wheel));
  ui->label_VehicleACC_Value->setText(QString::number(p.acceleration, 'f', 2));
  ui->label_VehicleSpeed_Value->setText(QString::number(p.velocity));
  ui->label_long_Value->setText(QString::number(p.longitude, 'f', 5));
  ui->label_lat_Value->setText(QString::number(p.latitude, 'f', 5));
  ui->label_Elevation_Value->setText(QString::number(p.altitude, 'f', 5));
  ui->label_Heading_Value->setText(QString::number(p.heading, 'f', 2));

  switch (p.gear_status) {
    case eGearPosition::GEARPOSITION_PARK:    ui->label_VehicleGear_Value->setText("P"); break;
    case eGearPosition::GEARPOSITION_REVERSE: ui->label_VehicleGear_Value->setText("R"); break;
    case eGearPosition::GEARPOSITION_NEUTRAL: ui->label_VehicleGear_Value->setText("N"); break;
    case eGearPosition::GEARPOSITION_DRIVE:   ui->label_VehicleGear_Value->setText("D"); break;
  }

  if      (p.vehicle_mode_status == 0) ui->label_RemoteStatus_Value->setText("Manual");
  else if (p.vehicle_mode_status == 1) ui->label_RemoteStatus_Value->setText("Auto");
  else if (p.vehicle_mode_status == 2) ui->label_RemoteStatus_Value->setText("Remote");

  // Lanelet2: 포즈만 갱신하고 리렌더는 CoalescedTimer로 상한
  if (lanelet_canvas_) {
    lanelet_canvas_->setVehiclePoseGeo(p.latitude, p.longitude, p.heading);
    laneletFlush_->poke(); // 50ms 내 한 번만 update()
  }

  // WebEngine 지도: 10Hz + 델타 임계치
  if (web_map_enabled_ && map_bridge_ && carpos_rl_.allow()) {
    const double dLat = p.latitude  - last_web_lat_;
    const double dLon = p.longitude - last_web_lon_;
    const double meters_per_deg_lat = 111000.0;
    const double meters_per_deg_lon = std::cos(p.latitude*M_PI/180.0) * 111000.0;
    const double dist_m = std::sqrt((dLat*meters_per_deg_lat)*(dLat*meters_per_deg_lat) +
                                    (dLon*meters_per_deg_lon)*(dLon*meters_per_deg_lon));
    const double dHead = std::fabs(p.heading - last_web_heading_);
    if (dist_m >= kMinDeltaMeter_ || dHead >= kMinDeltaHeading_) {
      emit map_bridge_->setCarPosition(p.longitude, p.latitude, p.heading,
                                       QString::fromStdString(p.vehicle_id));
      last_web_lat_ = p.latitude;
      last_web_lon_ = p.longitude;
      last_web_heading_ = p.heading;
    }
  }

  // 차트 포인트
  const double t = QDateTime::currentMSecsSinceEpoch()/1000.0;
  QMetaObject::invokeMethod(this, [=](){
    add_point(metrics_.at("car_speed").graph, t, static_cast<double>(p.velocity));
    add_point(metrics_.at("steering").graph,  t, static_cast<double>(p.steering_wheel));
  }, Qt::QueuedConnection);
}

// ---------- V2X 통계 ----------
void v2xManagerWindow::get_v2x_stats(const nr_v2x_msgs::V2XStat& s) {
  ui->lineEdit_Latency->setText(QString::number(s.modem_latency / 100));
  ui->label_LatencyTotal_Value->setText(QString::number(s.e2e_latency));
  ui->label_LatencyMtoM_Value->setText(QString::number(s.modem_latency / 100)); // (us->ms 가정)
  ui->lineEdit_Distance->setText(QString::number(s.distance));
  ui->label_PDR_Value->setText(QString::number(s.ssov_pdr));
  
  double sum = 0.0;
  int count = 0;
  for (const auto &v : s.video_pdr) {   // std::array / std::vector 모두 range-for 가능
    sum += static_cast<double>(v);
    ++count;
  }
  int video_pdr_avg = 0;
  if (count > 0) {
    video_pdr_avg = static_cast<int>(std::lround(sum / static_cast<double>(count)));
    video_pdr_avg = std::max(0, std::min(100, video_pdr_avg)); // 0~100 클램프
  }
  ui->label_Video_PDR_Value->setText(QString::number(video_pdr_avg));


  const double t = QDateTime::currentMSecsSinceEpoch()/1000.0;
  QMetaObject::invokeMethod(this, [=](){
    add_point(metrics_.at("latency").graph, t, static_cast<double>(s.e2e_latency));
    add_point(metrics_.at("PDR").graph, t, static_cast<double>(s.ssov_pdr));
  }, Qt::QueuedConnection);
}

// ---------- 모뎀 ----------
void v2xManagerWindow::get_modem_status(const nr_v2x_msgs::ModemRxStatus& rx,
                                        const nr_v2x_msgs::ModemTxStatus& tx) {
  if (ui->comboBox_v2xStatus->currentText().toStdString() != "Modem") return;

  ui->label_TX_Device_Value->setText(QString::number(tx.dev_id));
  ui->label_TX_HW_Value->setText(QString::number(tx.hw_ver));
  ui->label_TX_SW_Value->setText(QString::number(tx.sw_ver));
  ui->label_TX_Time_Value->setText(QString::number(tx.device_timestamp));
  ui->label_TX_Power_Value->setText(QString::number(tx.tx_power));
  ui->label_TX_freq_Value->setText(QString::number(tx.freq));
  ui->label_TX_bandwidth_Value->setText(QString::number(tx.bandwidth));
  ui->label_TX_SCS_Value->setText(QString::number(tx.scs));
  ui->label_TX_MCS_Value->setText(QString::number(tx.mcs));
  ui->label_TX_lat_Value->setText(QString::number(tx.latitude, 'f', 6));
  ui->label_TX_long_Value->setText(QString::number(tx.longitude, 'f', 6));

  ui->label_RX_Device_Value->setText(QString::number(rx.dev_id));
  ui->label_RX_HW_Value->setText(QString::number(rx.hw_ver));
  ui->label_RX_SW_Value->setText(QString::number(rx.sw_ver));
  ui->label_RX_Time_Value->setText(QString::number(rx.device_timestamp));
  ui->label_RX_RSSI_Value->setText(QString::number(rx.rssi));
  ui->label_RX_RCPI_Value->setText(QString::number(rx.rcpi));
  ui->label_RX_lat_Value->setText(QString::number(rx.latitude, 'f', 6));
  ui->label_RX_long_Value->setText(QString::number(rx.longitude, 'f', 6));
}

// ---------- 통신장치(OBU/RSU) ----------
void v2xManagerWindow::get_com_status(const nr_v2x_msgs::CommUnitStatus& cs) {
  if (ui->comboBox_v2xStatus->currentText().toStdString() == "Modem") return;

  if (cs.dev_type == 11) {
    ui->groupBox_TX_status->setTitle("TX-OBU Comm Status");
    ui->groupBox_RX_status->setTitle("RX-OBU Comm Status");
  } else if (cs.dev_type == 21) {
    ui->groupBox_TX_status->setTitle("TX-RSU Comm Status");
    ui->groupBox_RX_status->setTitle("RX-RSU Comm Status");
  } else {
    ui->groupBox_TX_status->setTitle("TX- Comm Status");
    ui->groupBox_RX_status->setTitle("RX- Comm Status");
  }

  if (cs.tx_rx == 0) {
    ui->label_TX_Device_Value->setText(QString::number(cs.dev_id));
    ui->label_TX_HW_Value->setText(QString::number(cs.hw_ver));
    ui->label_TX_SW_Value->setText(QString::number(cs.sw_ver));
    ui->label_TX_Time_Value->setText(QString::number(cs.device_timestamp));
    ui->label_TX_Power_Value->setText("NULL");
    ui->label_TX_freq_Value->setText("NULL");
    ui->label_TX_bandwidth_Value->setText("NULL");
    ui->label_TX_SCS_Value->setText("NULL");
    ui->label_TX_MCS_Value->setText("NULL");
    ui->label_TX_lat_Value->setText("NULL");
    ui->label_TX_long_Value->setText("NULL");
  } else {
    ui->label_RX_Device_Value->setText(QString::number(cs.dev_id));
    ui->label_RX_HW_Value->setText(QString::number(cs.hw_ver));
    ui->label_RX_SW_Value->setText(QString::number(cs.sw_ver));
    ui->label_RX_Time_Value->setText(QString::number(cs.device_timestamp));
    ui->label_RX_RSSI_Value->setText("NULL");
    ui->label_RX_RCPI_Value->setText("NULL");
    ui->label_RX_lat_Value->setText("NULL");
    ui->label_RX_long_Value->setText("NULL");
  }
}

// ---------- 네트워크 ----------
const tcpip_msgs::LinkMetrics*
v2xManagerWindow::findMetricsBySocketId(const tcpip_msgs::status& st, uint32_t socketId) const {
  for (const auto& m : st.socket_metrics) {
    if (m.socket_id == socketId) return &m;
  }
  return nullptr;
}
void v2xManagerWindow::change_network_status(const tcpip_msgs::status& st)
{
  gui_network_msg = st;

// Prune removed sockets from the cached set and UI lists
{
  QSet<uint32_t> current;
  for (const auto& s : st.tcp_sockets) current.insert(s.id);

  // remove from cached set anything not present
  for (auto it = listed_socket_ids_.begin(); it != listed_socket_ids_.end(); ) {
    if (!current.contains(*it)) {
      // remove any matching items from both lists
      auto removeById = [&](QListWidget* lw){
        for (int row = lw->count()-1; row >= 0; --row) {
          const auto txt = lw->item(row)->text();
          if (txt.startsWith(QString::number(*it) + " - ")) {
            delete lw->takeItem(row);
          }
        }
      };
      removeById(ui->listWidget_Server);
      removeById(ui->listWidget_Client);
      it = listed_socket_ids_.erase(it);
    } else {
      ++it;
    }
  }
}


  // ip 배열은 boost::array<uint8_t,4>
  auto fmtIp = [](const boost::array<uint8_t,4>& ip) -> QString {
    return QString("%1.%2.%3.%4").arg(ip[0]).arg(ip[1]).arg(ip[2]).arg(ip[3]);
  };

  // === Vehicle(Client) 모드: Server 리스트는 tcp_clients 기준 ===
  if (mode_status == 0) {
    for (const auto& c : st.tcp_clients) {
      if (listed_socket_ids_.contains(c.id)) continue;
      if (c.remote_endpoint.port == 0) continue; // 아직 원격 포트 채워지지 않은 상태면 스킵

      const QString ipport = fmtIp(c.remote_endpoint.ip) + ":" + QString::number(c.remote_endpoint.port);
      const QString item   = QString::number(c.id) + " - " + ipport;

      bool exists = false;
      for (int i = 0; i < ui->listWidget_Server->count(); ++i) {
        if (ui->listWidget_Server->item(i)->text() == item) { exists = true; break; }
      }
      if (!exists) {
        ui->listWidget_Server->addItem(item);
        listed_socket_ids_.insert(c.id);
      }
    }
  }
  // === Operator(Server) 모드: Client 리스트는 tcp_sockets 기준 ===
  else {
    for (const auto& s : st.tcp_sockets) {
      if (listed_socket_ids_.contains(s.id)) continue;
      if (s.remote_endpoint.port == 0) continue;

      const QString ipport = fmtIp(s.remote_endpoint.ip) + ":" + QString::number(s.remote_endpoint.port);
      const QString item   = QString::number(s.id) + " - " + ipport;

      bool exists = false;
      for (int i = 0; i < ui->listWidget_Client->count(); ++i) {
        if (ui->listWidget_Client->item(i)->text() == item) { exists = true; break; }
      }
      if (!exists) {
        ui->listWidget_Client->addItem(item);
        listed_socket_ids_.insert(s.id);
      }
    }
  }

  // === 통계 갱신 ===
  if (!st.tcp_sockets.empty()) {
    const uint32_t sid = st.tcp_sockets.back().id;
    if (const auto* met = findMetricsBySocketId(st, sid)) {
      const double txMbps = met->tx_bps / 1e6;
      const double rxMbps = met->rx_bps / 1e6;
      ui->label_SendBytesPer_Value->setText(QString::number(txMbps, 'f', 2) + " Mbps");
      ui->label_RecvBytesPer_Value->setText(QString::number(rxMbps, 'f', 2) + " Mbps");
      ui->lineEdit_Bitrate->setText(QString::number(rxMbps * 1000.0, 'f', 0)); // kbps

      const double pdrPct = std::clamp(met->pdr_rx * 100.0, 0.0, 100.0);
      const double t = QDateTime::currentMSecsSinceEpoch() / 1000.0;
      QMetaObject::invokeMethod(this, [=](){
        add_point(metrics_.at("PDR").graph, t, pdrPct);
      }, Qt::QueuedConnection);
    }
  }
}



// ---------- Lanelet2: 파일 열기/로드 ----------
static std::string tryExtractProj4(const QString& osmPath) {
  QFile f(osmPath);
  if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) return {};
  const QString xml = QString::fromUtf8(f.readAll());
  f.close();
  QRegExp rx1("k\\s*=\\s*\"geo(reference|Reference)\"\\s+v\\s*=\\s*\"([^\"]+)\"");
  QRegExp rx2("<geoReference>([^<]+)</geoReference>");
  if (rx1.indexIn(xml) >= 0) return rx1.cap(2).toStdString();
  if (rx2.indexIn(xml) >= 0) return rx2.cap(1).toStdString();
  return {};
}

void v2xManagerWindow::open_lanelet2_map() {
  const QString file = QFileDialog::getOpenFileName(
      this, tr("Open Autoware Lanelet2 Map (.osm)"),
      QDir::homePath(),
      tr("OSM Files (*.osm *.xml)"));
  if (file.isEmpty()) return;

  ui->stackedWidget->setCurrentIndex(1);
  ui->menuLabel->setText("Lanelet2 Map");

  if (!lanelet_canvas_) {
    QMessageBox::warning(this, "Error", "Lanelet canvas not ready");
    return;
  }

  const std::string proj4 = tryExtractProj4(file);

  bool ok = false;
  if (!proj4.empty()) {
    lanelet::Origin dummy; // isDefault=true
    ok = lanelet_canvas_->loadLanelet2(file, proj4, dummy);
  } else {
    lanelet::Origin origin(lanelet::GPSPoint{36.7278803, 127.4430473, 0.0});
    ok = lanelet_canvas_->loadLanelet2(file, /*geoRef*/"", origin);
  }

  if (!ok) {
    QMessageBox::warning(this, "Error", "Failed to load Lanelet2 map");
    return;
  }

  lanelet_canvas_->setVehiclePoseGeo(36.7278803, 127.4430473, 0.0);
  lanelet_canvas_->centerOnVehicle();
}

bool v2xManagerWindow::loadLanelet2IntoCanvas(const QString& file) {
  if (file.isEmpty()) return false;

  int pageIdx = ui->stackedWidget->indexOf(ui->mapWidget->parentWidget());
  if (pageIdx >= 0) {
    ui->stackedWidget->setCurrentIndex(pageIdx);
    ui->menuLabel->setText("Lanelet2 Map");
  }

  if (!lanelet_canvas_) {
    QMessageBox::warning(this, "Error", "Lanelet canvas not ready");
    return false;
  }

  const std::string proj4 = tryExtractProj4(file);
  bool ok = false;
  if (!proj4.empty()) {
    lanelet::Origin dummy; // default
    ok = lanelet_canvas_->loadLanelet2(file, proj4, dummy);
  } else {
    lanelet::Origin origin(lanelet::GPSPoint{36.7278803, 127.4430473, 0.0});
    ok = lanelet_canvas_->loadLanelet2(file, /*geoRef*/"", origin);
  }

  if (!ok) return false;

  lanelet_canvas_->setVehiclePoseGeo(36.7278803, 127.4430473, 0.0);
  lanelet_canvas_->centerOnVehicle();
  return true;
}

// ---------- init_map (지연 로드 정책 — 필요 시만 수동 호출) ----------
void v2xManagerWindow::init_map() {
  // 이전 버전과 호환을 위해 남겨둠. 기본적으로는 setWebMapEnabled_(true)로 지연 로드 권장.
  if (!web_map_loaded_) setWebMapEnabled_(true);
}