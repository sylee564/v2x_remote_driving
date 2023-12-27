#ifndef V2XMANAGERWINDOW_H
#define V2XMANAGERWINDOW_H

#include <QMainWindow>
#include <QWidget>
#include <QtCore/QString>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QPushButton>
#include <QtNetwork/QHostAddress>
#include <QtCore/QTimer>
#include <QtCore/QMap>
#include <QtCore/QString>
#include <QtCore>
#include <QDebug>
#include <string>
#include <ros/package.h>
#include "tod_msgs/inputDevice.h"
#include "tod_msgs/Status.h"
#include "tod_msgs/ControlCmd.h"
#include "tod_msgs/PVD_data.h"
#include "pvd_msgs/PVD_data.h"

#include <memory>
#include <map>
#include <chrono>
#include <thread>
#include <vector>
#include <mutex>

#include <stdio.h>
#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>
#include "QtWidgetGroup.h"


QT_BEGIN_NAMESPACE
namespace Ui { class v2xManagerWindow; }
QT_END_NAMESPACE

inline std::string PRIO4 { "127.0.0." };
inline std::string PRIO5 { "192.168." };
inline std::vector<std::string> listWithPriorities;

bool sortIpAddresses(const std::string& T1, const std::string& T2);

class v2xManagerWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit v2xManagerWindow(const std::string& pathToYamlFile,
                const std::string& searchedKey, QObject* operatorManager,  QWidget *parent = nullptr);
    ~v2xManagerWindow();

    void change_button_status_after_clicked_on_connect(bool conFlag);
    void set_ComboBox_IpAddressPC_Text(const std::string& ip_addr);
    void set_ComboBox_IpAddressDevice_Text(const std::string& ip_addr);
    tod_msgs::Status get_gui_status();
    // tod_msgs::ControlCmd get_gui_controlCmd();
    void set_gui_connection_status_to(const uint8_t connection_status);
    void set_gui_status_control_mode(const uint8_t control_mode);
    void set_gui_status_v2x_header(const std::vector<uint16_t> v2x_header);
    void update_safety_driver_status_labels();

    Ui::v2xManagerWindow *ui;

public slots:
    // necessary in order to close application from STRG + C (cli)
    void quitAll();

    void on_statusBtn_clicked();
    void on_dataBtn_clicked();
    void on_reportBtn_clicked();
    void on_settingBtn_clicked();
    void on_infoBtn_clicked();
    void on_connectBtn_clicked();
    void on_radioButton_Vehicle_clicked();
    void on_StartAndStopBtn_clicked();
    void on_radioButton_Operator_clicked();
    void on_BrowseAndOkBtn_clicked();
    void on_PushButton_DirectControl_clicked();
    void on_PushButton_IndirectControl_clicked();
    void on_PushButton_VideoMode_Single_clicked();
    void on_PushButton_VideoMode_Multiple_clicked();
    void change_emergency_stop_released(bool released);
    void EPS_approved(bool approved);
    void ACC_approved(bool approved);
    void get_control_command_data(tod_msgs::ControlCmd contorl_data);
    void get_pvd_data(tod_msgs::PVD_data pvd_data);

signals:
    void signal_on_connectBtn_Connect_clicked(const std::string& ip_addr_PC, const std::string& ip_addr_device,
                                      const int ip_device_port, const std::vector<uint16_t> v2x_header);
    void signal_on_connectBtn_Disconnect_clicked();
    void signal_on_StartAndStopBtn_Start_clicked();
    void signal_on_StartAndStopBtn_Stop_clicked();
    void signal_control_mode_changed(const uint8_t control_mode);
    void signal_input_device_changed(const std::string& input_device);
    void signal_video_mode_changed(const uint8_t video_mode);
    void signal_on_radioBtn_Select_clicked(const uint8_t mode_status);

private:
    std::string _pathToYamlFile;
    uint8_t mode_status{0};

    std::vector<std::string> _listOfIpAddressesPC;

    std::vector<std::string> ipAddresses;
    std::vector<std::string> portNumbers;
    std::map<int, std::string> deviceTypes;
    std::map<int, std::string> serviceIDs;
    std::map<int, std::string> telecomTypes;
    std::map<int, std::string> regionIDs;
    std::map<int, std::string> actionTypes;
    std::map<int, std::string> comIDs;
    std::map<int, std::string> payloadTypes;

    std::vector<uint16_t> headerList;

    void fillComboBox(std::vector<std::string> configNodes);
    void update_header();
    void setVehicleComboBox();
    void setOperatorComboBox();
    void readAndStoreOwnIpAddresses();
    void addIpAddressesToComboBox();
    

    QtWidgetGroup control_buttons;
    QtWidgetGroup video_mode_buttons;
    void register_control_mode_buttons();
    void register_video_mode_buttons();

    tod_msgs::Status gui_status;
    tod_msgs::ControlCmd gui_controlCmd;
    tod_msgs::PVD_data gui_pvdData;
    
    void init_gui_status(); 

};
#endif // V2XMANAGERWINDOW_H
