#include <algorithm>
#include "v2xManagerWindow.h"
#include "ui_v2xManagerWindow.h"
#include "v2xHeaderDeserializer.h"

#define DeviceId 00000001
#define DbVer 0x0001
#define HwVer 0x0001
#define SwVer 0x0001


const std::string IpAddressKey="OBU-IpAddress";
const std::string PortKey="OBU-Port";
const std::string DeviceTypeKey="OBU-DeviceType";
const std::string ServiceIDKey="OBU-ServiceID";
const std::string TelecomTypeKey="OBU-TelecomType";
const std::string RegionIDKey="OBU-RegionID";
const std::string ActionTypeKey="OBU-ActionType";
const std::string CommunicationIDKey="OBU-CommunicationID";
const std::string PayloadTypeKey="OBU-PayloadType";

QString backgroundGreen("background-color:rgb(154,205,50)");
QString backgroundRed("background-color:rgb(205,92,92)");

v2xManagerWindow::v2xManagerWindow(const std::string& pathToYamlFile, 
    const std::string& searchedKey, QObject* operatorManager, QWidget *parent)
    : _pathToYamlFile(pathToYamlFile),QMainWindow(parent)
    , ui(new Ui::v2xManagerWindow)
{
    ui->setupUi(this);

    // register grouped buttons
    register_control_mode_buttons();
    register_video_mode_buttons();

    // enable only the steps that are possible in "idle" state
    control_buttons.enableButtons(false);
    video_mode_buttons.enableButtons(false);

    std::vector<std::string> configNodes=v2xHeaderDeserializer::vectorLoad(_pathToYamlFile, searchedKey);
    fillComboBox(configNodes);
    readAndStoreOwnIpAddresses();
    addIpAddressesToComboBox();

    ui->BrowseAndOkBtn->setEnabled(false);
    ui->LineEdit_PathToInputDevice->setEnabled(false);
    ui->StartAndStopBtn->setEnabled(false);
    ui->radioButton_Vehicle->setChecked(true);
    on_radioButton_Vehicle_clicked();

    init_gui_status();
    ui->Label_EmergencyBreakReleased->setStyleSheet(backgroundRed);
    ui->Label_ACCReleased->setStyleSheet(backgroundRed);
    ui->Label_EPSReleased->setStyleSheet(backgroundRed);
    this->show();
}

v2xManagerWindow::~v2xManagerWindow()
{
    delete ui;
}

// gui status initialization
void v2xManagerWindow::init_gui_status() {
    gui_status.mode_status=0;
    gui_status.tod_status = tod_msgs::Status::TOD_STATUS_IDLE;
    gui_status.vehicle_EPS_approved = false;
    gui_status.vehicle_ACC_approved = false;
    gui_status.vehicle_EPS_Status = false;
    gui_status.vehicle_ACC_Status = false;
    gui_status.vehicle_emergency_stop_released = false;
    gui_status.vehicle_device_ip_address = "127.0.0.1";

    gui_status.operator_control_mode =
        tod_msgs::Status::CONTROL_MODE_DIRECT;
    gui_status.operator_video_mode =
        tod_msgs::Status::VIDEO_RATE_CONTROL_MODE_SINGLE;
    gui_status.operator_input_device =
        InputDevice::VIRTUALINPUTDEVICE;
}

// register control mode button
void v2xManagerWindow::register_control_mode_buttons() {
    control_buttons.addWidget(ui->PushButton_DirectControl);
    control_buttons.addWidget(ui->PushButton_IndirectControl);
}

// register video mode button
void v2xManagerWindow::register_video_mode_buttons() {
    video_mode_buttons.addWidget(ui->PushButton_VideoMode_Single);
    video_mode_buttons.addWidget(ui->PushButton_VideoMode_Multiple);
}

// Menu bar click event
void v2xManagerWindow::on_statusBtn_clicked()
{
    ui->stackedWidget->setCurrentIndex(0);
    ui->menuLabel->setText("Status");
}

void v2xManagerWindow::on_dataBtn_clicked()
{
    ui->stackedWidget->setCurrentIndex(1);
    ui->menuLabel->setText("Data Analysis");
}

void v2xManagerWindow::on_reportBtn_clicked()
{
    ui->stackedWidget->setCurrentIndex(2);
    ui->menuLabel->setText("Reports");
}

void v2xManagerWindow::on_settingBtn_clicked()
{
    ui->stackedWidget->setCurrentIndex(3);
    ui->menuLabel->setText("Setting");
}

void v2xManagerWindow::on_infoBtn_clicked()
{
    ui->stackedWidget->setCurrentIndex(4);
    ui->menuLabel->setText("Information");
}

// Fill the combobox with parameter 
void v2xManagerWindow::fillComboBox(std::vector<std::string> configNodes)
{
    for(auto configNode:configNodes){
        if(configNode==IpAddressKey)
        {
            ipAddresses= v2xHeaderDeserializer::vectorLoad(_pathToYamlFile, configNode);
            for(auto ipAddress : ipAddresses){
                ui->ComboBox_IpAddressOBU->addItem(QString::fromStdString(ipAddress));
            }
        }
        else if(configNode==PortKey)
        {
            portNumbers=v2xHeaderDeserializer::vectorLoad(_pathToYamlFile, configNode);
            for(auto portNumber : portNumbers){
                ui->ComboBox_PortOBU->addItem(QString::fromStdString(portNumber));
            }

        }
        else if(configNode==DeviceTypeKey)
        {
            deviceTypes=v2xHeaderDeserializer::mapLoad(_pathToYamlFile, configNode);
            for(auto deviceType : deviceTypes){
                ui->ComboBox_DeviceType->addItem(QString::fromStdString(deviceType.second));
            }
        }
        else if(configNode==ServiceIDKey)
        {
            serviceIDs=v2xHeaderDeserializer::mapLoad(_pathToYamlFile, configNode);
            for(auto serviceID : serviceIDs){
                ui->ComboBox_ServiceID->addItem(QString::fromStdString(serviceID.second));
            }
        }
        else if(configNode==TelecomTypeKey)
        {
            telecomTypes=v2xHeaderDeserializer::mapLoad(_pathToYamlFile, configNode);
            for(auto telecomType : telecomTypes){
                 ui->ComboBox_TelecomType->addItem(QString::fromStdString(telecomType.second));
            }
        }
        else if(configNode==RegionIDKey)
        {
            regionIDs=v2xHeaderDeserializer::mapLoad(_pathToYamlFile, configNode);
            for(auto regionID : regionIDs){
                ui->ComboBox_RegionID->addItem(QString::fromStdString(regionID.second));
            }
        }
        else if(configNode==ActionTypeKey)
        {
            actionTypes=v2xHeaderDeserializer::mapLoad(_pathToYamlFile, configNode);
            for(auto actionType : actionTypes){
                ui->ComboBox_ActionType->addItem(QString::fromStdString(actionType.second));
            }
        }
        else if(configNode==CommunicationIDKey)
        {
            comIDs=v2xHeaderDeserializer::mapLoad(_pathToYamlFile, configNode);
            for(auto comID : comIDs){
                 ui->ComboBox_ComID->addItem(QString::fromStdString(comID.second));
             }
        }
        else if(configNode==PayloadTypeKey)
        {
            payloadTypes=v2xHeaderDeserializer::mapLoad(_pathToYamlFile, configNode);
            for(auto payloadType : payloadTypes){
                ui->ComboBox_PayloadType->addItem(QString::fromStdString(payloadType.second));
            }
        }
    }
}

// Set operator combobox
void v2xManagerWindow::setOperatorComboBox()
{
    ui->ComboBox_DeviceType->setCurrentText(QString(deviceTypes[2].c_str()));
    ui->ComboBox_ServiceID->setCurrentText(QString(serviceIDs[3].c_str()));
    ui->ComboBox_TelecomType->setCurrentText(QString(telecomTypes[2].c_str()));
    ui->ComboBox_ActionType->setCurrentText(QString(actionTypes[2].c_str()));
    ui->ComboBox_RegionID->setCurrentText(QString(regionIDs[8].c_str()));
    ui->ComboBox_ComID->setCurrentText(QString(comIDs[5].c_str()));
    ui->ComboBox_PayloadType->setCurrentText(QString(payloadTypes[5].c_str()));
}

// Set vehicle combobox
void v2xManagerWindow::setVehicleComboBox()
{
    ui->ComboBox_DeviceType->setCurrentText(QString(deviceTypes[1].c_str()));
    ui->ComboBox_ServiceID->setCurrentText(QString(serviceIDs[3].c_str()));
    ui->ComboBox_TelecomType->setCurrentText(QString(telecomTypes[2].c_str()));
    ui->ComboBox_ActionType->setCurrentText(QString(actionTypes[1].c_str()));
    ui->ComboBox_RegionID->setCurrentText(QString(regionIDs[8].c_str()));
    ui->ComboBox_ComID->setCurrentText(QString(comIDs[2].c_str()));
    ui->ComboBox_PayloadType->setCurrentText(QString(payloadTypes[2].c_str()));
}

// Read and store owner PC Ip Addresses 
void v2xManagerWindow::readAndStoreOwnIpAddresses() {
    struct ifaddrs * ifAddrStruct = NULL;
    struct ifaddrs * ifa = NULL;
    void * tmpAddrPtr = NULL;

    getifaddrs(&ifAddrStruct);

    for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr) {
            continue;
        }
        if (ifa->ifa_addr->sa_family == AF_INET) { // check it is IP4
            // is a valid IP4 Address
            struct sockaddr_in * tempPointer = (struct sockaddr_in*)(ifa->ifa_addr);
            tmpAddrPtr = &(tempPointer->sin_addr);
            char addressBuffer[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
            printf("%s IP Address %s\n", ifa->ifa_name, addressBuffer);
            std::string ip(addressBuffer);
            // store the ipv4 addresses
            _listOfIpAddressesPC.push_back(ip);
        } else if (ifa->ifa_addr->sa_family == AF_INET6) { // check it is IP6
            // is a valid IP6 Address
            struct sockaddr_in6 * tempPointer = (struct sockaddr_in6*)(ifa->ifa_addr);
            tmpAddrPtr = &(tempPointer->sin6_addr);
            char addressBuffer[INET6_ADDRSTRLEN];
            inet_ntop(AF_INET6, tmpAddrPtr, addressBuffer, INET6_ADDRSTRLEN);
            printf("%s IP Address %s\n", ifa->ifa_name, addressBuffer);
        }
    }

    if (ifAddrStruct != NULL) freeifaddrs(ifAddrStruct);

    // sort my ip addresses:
    listWithPriorities.push_back(PRIO4);
    listWithPriorities.push_back(PRIO5);

    std::sort(_listOfIpAddressesPC.begin(), _listOfIpAddressesPC.end(), sortIpAddresses);
}

// Add Ip Addressed to combobox
void v2xManagerWindow::addIpAddressesToComboBox()
{
    for ( auto it = _listOfIpAddressesPC.begin(); it != _listOfIpAddressesPC.end(); ++it ) {
        ui->ComboBox_IpAddressPC->addItem((QString::fromLocal8Bit)((*it).c_str()));
    }
}

// Sort Ip Addresses
bool sortIpAddresses(const std::string& T1, const std::string& T2) {
    // if PRIO1 is found in T1 -> no work to do -> return true;
    // otherwise, sort is triggered with false
    for ( auto it = listWithPriorities.begin(); it != listWithPriorities.end(); ++it ) {
        size_t t1 = T1.find(*it);
        size_t t2 = T2.find(*it);
        if (t1 != std::string::npos) {
            return true;
        } else if ( t2 != std::string::npos ) {
            return false;
        }
    }
    return true;
}

void v2xManagerWindow::change_button_status_after_clicked_on_connect(bool conFlag)
{
    // setting enable or disable
    ui->ComboBox_IpAddressOBU->setEnabled(!conFlag);
    ui->ComboBox_PortOBU->setEnabled(!conFlag);
    ui->ComboBox_DeviceType->setEnabled(!conFlag);
    ui->ComboBox_ServiceID->setEnabled(!conFlag);
    ui->ComboBox_TelecomType->setEnabled(!conFlag);
    ui->ComboBox_RegionID->setEnabled(!conFlag);
    ui->ComboBox_ActionType->setEnabled(!conFlag);
    ui->ComboBox_ComID->setEnabled(!conFlag);
    ui->ComboBox_PayloadType->setEnabled(!conFlag);
    if(gui_status.vehicle_ACC_approved == 1 && gui_status.vehicle_EPS_approved == 1){
        ui->StartAndStopBtn->setEnabled(conFlag);
    }
    else{
        ui->StartAndStopBtn->setEnabled(false);
    }
    ui->StartAndStopBtn->setEnabled(conFlag);
    ui->radioButton_Operator->setEnabled(!conFlag);
    ui->radioButton_Vehicle->setEnabled(!conFlag);

    if(gui_status.mode_status==0){
        control_buttons.enableButtons(false);
        video_mode_buttons.enableButtons(false);
        ui->StartAndStopBtn->setEnabled(false);
    }else{
        control_buttons.enableButtons(conFlag);
        video_mode_buttons.enableButtons(conFlag);
    }
    
    // ui->groupBox_Mode->setEnabled(!conFlag);

    if(conFlag){
        update_header();
    }
    else{
        headerList.clear();
    }
}

// Connect button click event
void v2xManagerWindow::on_connectBtn_clicked()
{
    if(ui->connectBtn->text()=="Connect"){
        ui->connectBtn->setText("Disconnect");
        change_button_status_after_clicked_on_connect(true);
        // ui->StartAndStopBtn->setEnabled(true);
        ui->Label_connectState->setText("Uplink...");
        // update_header();
        set_gui_connection_status_to(tod_msgs::Status::TOD_STATUS_UPLINK_ONLY);
        emit signal_on_connectBtn_Connect_clicked(
            ui->ComboBox_IpAddressPC->currentText().toStdString(),
            ui->ComboBox_IpAddressOBU->currentText().toStdString(),
            ui->ComboBox_PortOBU->currentText().toInt(), headerList);
        // std::cout<< "port number : "<< ui->ComboBox_PortOBU->currentText().toInt()<<std::endl;
    }
    else{
        ui->connectBtn->setText("Connect");
        change_button_status_after_clicked_on_connect(false);
        ui->StartAndStopBtn->setText("Start");
        ui->Label_connectState->setText("Ready...");
        set_gui_connection_status_to(tod_msgs::Status::TOD_STATUS_IDLE);
        emit signal_on_connectBtn_Disconnect_clicked();
    }
}

// header list updata
void v2xManagerWindow::update_header()
{
    if(headerList.empty()){
        for(auto deviceType : deviceTypes){
            if(deviceType.second == ui->ComboBox_DeviceType->currentText().toStdString()){
                headerList.emplace_back(deviceType.first);
            }
        }
        for(auto telecomType : telecomTypes){
            if(telecomType.second == ui->ComboBox_TelecomType->currentText().toStdString())
                headerList.emplace_back(telecomType.first);
        }
        headerList.emplace_back(DeviceId);
        for(auto serviceID : serviceIDs){
            if(serviceID.second == ui->ComboBox_ServiceID->currentText().toStdString())
                headerList.emplace_back(serviceID.first);
        }
        for(auto actionType : actionTypes){
            if(actionType.second == ui->ComboBox_ActionType->currentText().toStdString())
                headerList.emplace_back(actionType.first);
        }
        for(auto regionID : regionIDs){
            if(regionID.second == ui->ComboBox_RegionID->currentText().toStdString())
                headerList.emplace_back(regionID.first);
        }
    
        for(auto comID : comIDs){
            if(comID.second == ui->ComboBox_ComID->currentText().toStdString())
                headerList.emplace_back(comID.first);
        }
        for(auto payloadType : payloadTypes){
            if(payloadType.second == ui->ComboBox_PayloadType->currentText().toStdString())
                headerList.emplace_back(payloadType.first);
        }
        headerList.emplace_back(DbVer);
        headerList.emplace_back(HwVer);
        headerList.emplace_back(SwVer);
    }
    
}

// vehicle radio button click event
void v2xManagerWindow::on_radioButton_Vehicle_clicked()
{
    ui->BrowseAndOkBtn->setEnabled(false);
    ui->LineEdit_PathToInputDevice->setEnabled(false);
    setVehicleComboBox();
    // mode_status=0;
    gui_status.mode_status=0;
    control_buttons.enableButtons(false);
    video_mode_buttons.enableButtons(false);
    // emit signal_on_radioBtn_Select_clicked(mode_status);
//    ui->ComboBox_DeviceType->setCurrentText(QString(deviceTypes[1].c_str()));
//    ui->frame_inputDevice->setStyleSheet("background-color: rgb(136, 138, 133)");
}

// operator radio button click event
void v2xManagerWindow::on_radioButton_Operator_clicked()
{
    ui->BrowseAndOkBtn->setEnabled(true);
    ui->LineEdit_PathToInputDevice->setEnabled(true);
    setOperatorComboBox();
    // mode_status=1;
    gui_status.mode_status=1;
    // emit signal_on_radioBtn_Select_clicked(mode_status);
}

// Start and Stop button click event
void v2xManagerWindow::on_StartAndStopBtn_clicked()
{
    if(ui->StartAndStopBtn->text()=="Start"){
        ui->StartAndStopBtn->setText("Stop");
        ui->Label_connectState->setText("Teleoperation");
        control_buttons.enableButtons(false);
        // change_button_status_after_clicked_on_connect(true);
        set_gui_connection_status_to(tod_msgs::Status::TOD_STATUS_TELEOPERATION);
        emit signal_on_StartAndStopBtn_Start_clicked();
    }
    else{
        ui->StartAndStopBtn->setText("Start");
        ui->Label_connectState->setText("Uplink...");
        control_buttons.enableButtons(true);
        // change_button_status_after_clicked_on_connect(false);
        set_gui_connection_status_to(tod_msgs::Status::TOD_STATUS_UPLINK_ONLY);

        emit signal_on_StartAndStopBtn_Stop_clicked();
    }
}

// Browse and Ok button click event
void v2xManagerWindow::on_BrowseAndOkBtn_clicked()
{
    std::string expectedPath = ros::package::getPath("tod_input_devices") + "/config";
    std::string fileName = QFileDialog::getOpenFileName(this,
        tr("Open Input Device Config Files"),
        QString::fromStdString(expectedPath), tr("Config Files (*.yaml)")).toStdString();
    ui->LineEdit_PathToInputDevice->setText(QString::fromStdString(fileName));
    emit signal_input_device_changed(fileName);
}

// Direct control button click event
void v2xManagerWindow::on_PushButton_DirectControl_clicked() {
        emit signal_control_mode_changed(tod_msgs::Status::CONTROL_MODE_DIRECT);
        control_buttons.switchFocusTo(ui->PushButton_DirectControl, backgroundGreen);
        set_gui_status_control_mode(tod_msgs::Status::CONTROL_MODE_DIRECT);
    
}

// Indirect control button click event
void v2xManagerWindow::on_PushButton_IndirectControl_clicked() {
        emit signal_control_mode_changed(tod_msgs::Status::CONTROL_MODE_INDIRECT);
        control_buttons.switchFocusTo(ui->PushButton_IndirectControl, backgroundGreen);
        set_gui_status_control_mode(tod_msgs::Status::CONTROL_MODE_INDIRECT);
    
}

// Single video mode button click event
void v2xManagerWindow::on_PushButton_VideoMode_Single_clicked() {
    emit signal_video_mode_changed(tod_msgs::Status::VIDEO_RATE_CONTROL_MODE_SINGLE);
    video_mode_buttons.switchFocusTo(ui->PushButton_VideoMode_Single, backgroundGreen);
}

// multiple video mode button click event
void v2xManagerWindow::on_PushButton_VideoMode_Multiple_clicked() {
    emit signal_video_mode_changed(tod_msgs::Status::VIDEO_RATE_CONTROL_MODE_MULTIPLE);
    video_mode_buttons.switchFocusTo(ui->PushButton_VideoMode_Multiple, backgroundGreen);
}

// application quit
void v2xManagerWindow::quitAll() {
    QApplication::quit();
}


void v2xManagerWindow::set_ComboBox_IpAddressPC_Text(
        const std::string& ip_addr) {
    ui->ComboBox_IpAddressPC->setCurrentText(QString::fromStdString(ip_addr));
}

void v2xManagerWindow::set_ComboBox_IpAddressDevice_Text(
        const std::string& ip_addr) {
    ui->ComboBox_IpAddressOBU->setCurrentText(QString::fromStdString(ip_addr));
}

tod_msgs::Status v2xManagerWindow::get_gui_status() {
    return gui_status;
}

void v2xManagerWindow::set_gui_connection_status_to(const uint8_t connection_status) {
    gui_status.tod_status = connection_status;
}

void v2xManagerWindow::set_gui_status_control_mode(const uint8_t control_mode) {
    // if(gui_status.mode_status==1){
    //     gui_status.operator_control_mode = control_mode;
    // }
    // else{
    //     gui_status.vehicle_control_mode = control_mode;
    // }
    gui_status.operator_control_mode = control_mode;
}

void v2xManagerWindow::set_gui_status_v2x_header(const std::vector<uint16_t> v2x_header) {
    if(gui_status.mode_status==1){
        gui_status.oper_v2x_header = v2x_header;
    }
    else{
        gui_status.vehicle_v2x_header = v2x_header;
    }
    
}

void v2xManagerWindow::change_emergency_stop_released(bool released) {
    gui_status.vehicle_emergency_stop_released = released;
    update_safety_driver_status_labels();
}

void v2xManagerWindow::EPS_approved(bool approved) {
    gui_status.vehicle_EPS_approved = approved;
    update_safety_driver_status_labels();
}

void v2xManagerWindow::ACC_approved(bool approved) {
    gui_status.vehicle_ACC_approved = approved;
    update_safety_driver_status_labels();
}

// After get control command message, display gui vehicle status 
void v2xManagerWindow::get_control_command_data(tod_msgs::ControlCmd control_data) {

    gui_controlCmd = control_data;
    int oper_speed=static_cast<int>(control_data.velocity*3.60);
    int oper_steering=static_cast<int>(control_data.steeringWheelAngle* 180.0 / 3.1415);

    ui->lineEdit_OperID->setText(QString::number(control_data.operator_id));

    if(control_data.control_type==tod_msgs::Status::CONTROL_MODE_DIRECT){
        ui->lineEdit_ControlType->setText("Direct");
    }
    else if(control_data.control_type==tod_msgs::Status::CONTROL_MODE_INDIRECT){
        ui->lineEdit_ControlType->setText("Indirect");
    }

    ui->lineEdit_SteeringAng->setText(QString::number(oper_steering));
    
    
    switch(control_data.gearPosition){
        case 0:ui->lineEdit_OperGear->setText("P"); break;
        case 1:ui->lineEdit_OperGear->setText("R"); break;
        case 2:ui->lineEdit_OperGear->setText("N"); break;
        case 3:ui->lineEdit_OperGear->setText("D"); break;
    }

    ui->lineEdit_OperSpeed->setText(QString::number(oper_speed));
    ui->lineEdit_OperAcc->setText(QString::number(control_data.acceleration));

    if(control_data.remote_flag) ui->lineEdit_RemoteSignal->setText("ON");
    else ui->lineEdit_RemoteSignal->setText("OFF");

    if(control_data.streaming_flag) ui->lineEdit_StreamSignal->setText("ON");
    else ui->lineEdit_StreamSignal->setText("OFF");

    switch(control_data.indicator){
        case 1:ui->lineEdit_OperIndicator->setText("Emergency"); break;
        case 2:ui->lineEdit_OperIndicator->setText("Left"); break;
        case 4:ui->lineEdit_OperIndicator->setText("Right"); break;
        default:ui->lineEdit_OperIndicator->setText("OFF"); break;
    }

    // std::cout << "steering wheel : "<<control_data.steeringWheelAngle<<std::endl;
    // update_safety_driver_status_labels();
}

// After get pvd data message, display gui vehicle status 
void v2xManagerWindow::get_pvd_data(tod_msgs::PVD_data pvd_data) {
    gui_pvdData = pvd_data;
    ui->lineEdit_VehicleName->setText(QString::fromStdString(pvd_data.vehicle_name));
    ui->lineEdit_VehicleID->setText(QString::fromStdString(pvd_data.vehicle_id));
    ui->lineEdit_VehicleType->setText(QString::number(pvd_data.vehicle_type));
    ui->lineEdit_VehicleSteering->setText(QString::number(pvd_data.steering_wheel));
    ui->lineEdit_VehicleAcc->setText(QString::number(pvd_data.acceleration, 'f', 2));
    ui->lineEdit_VehicleSpeed->setText(QString::number(pvd_data.velocity));
    ui->lineEdit_Long->setText(QString::number(pvd_data.longitude, 'f', 10));
    ui->lineEdit_Lat->setText(QString::number(pvd_data.latitude, 'f', 10));
    ui->lineEdit_Elevation->setText(QString::number(pvd_data.elevation, 'f', 5));
    ui->lineEdit_Heading->setText(QString::number(pvd_data.heading, 'f', 2));
    
    switch(pvd_data.gear_status){
        case 0:ui->lineEdit_VehicleGear->setText("P"); break;
        case 1:ui->lineEdit_VehicleGear->setText("R"); break;
        case 2:ui->lineEdit_VehicleGear->setText("N"); break;
        case 3:ui->lineEdit_VehicleGear->setText("D"); break;
    }

    if(pvd_data.vehicle_mode==1) ui->lineEdit_RemoteStatus->setText("ON");
    else ui->lineEdit_RemoteStatus->setText("OFF");
}

// safety driver status label update
void v2xManagerWindow::update_safety_driver_status_labels() {
    if ( gui_status.vehicle_emergency_stop_released ) {
        ui->Label_EmergencyBreakReleased->setStyleSheet(backgroundGreen);
    } else {
        ui->Label_EmergencyBreakReleased->setStyleSheet(backgroundRed);
    }

    if ( gui_status.vehicle_EPS_approved ) {
        ui->Label_EPSReleased->setStyleSheet(backgroundGreen);
    } else {
        ui->Label_EPSReleased->setStyleSheet(backgroundRed);
    }

    if ( gui_status.vehicle_ACC_approved ) {
        ui->Label_ACCReleased->setStyleSheet(backgroundGreen);
    } else {
        ui->Label_ACCReleased->setStyleSheet(backgroundRed);
    }

    // if(gui_status.vehicle_ACC_approved && gui_status.vehicle_EPS_approved){
    //     ui->StartAndStopBtn->setEnabled(true);
    // }
    // else{
    //     ui->StartAndStopBtn->setEnabled(false);
    //     ui->StartAndStopBtn->setText("Start");
    //     ui->Label_connectState->setText("Uplink...");
    //     set_gui_connection_status_to(tod_msgs::Status::TOD_STATUS_UPLINK_ONLY);
    // }
}