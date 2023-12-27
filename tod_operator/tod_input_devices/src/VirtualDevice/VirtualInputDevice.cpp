#include <QtQuick/QQuickView>
#include <QtQuick/QQuickItem>
#include <QUrl>
#include "VirtualInputDevice.h"
#include "ui_joystickwindow.h"

VirtualInputDevice::VirtualInputDevice(std::function<void(const int, const double)> axisCb,
    std::function<void(const int, const int)> buttonCb, QWidget *parent)
    : QMainWindow(parent), InputDevice(axisCb, buttonCb) {

    ui = new Ui::JoystickWindow;
    ui->setupUi(this);
    ui->quickWidget->setSource(QUrl("qrc:///virtual_joystick.qml"));
    connect(ui->quickWidget->rootObject(), SIGNAL(userInput(double, double)),
        this, SLOT(on_userInput(double, double)));

    connect(this, &VirtualInputDevice::activateSignal, this, &VirtualInputDevice::on_activate);
    connect(this, &VirtualInputDevice::deactivateSignal, this, &VirtualInputDevice::on_deactivate);
    connect(this, &VirtualInputDevice::terminateSignal, this, &VirtualInputDevice::on_terminate);
    installEventFilter(this);
}

VirtualInputDevice::~VirtualInputDevice() {
    delete ui;
    this->close();
    //QApplication::quit();
}

bool VirtualInputDevice::activate() {
    emit activateSignal();
    return true;
}

bool VirtualInputDevice::deactivate() {
    emit deactivateSignal();
    return true;
}

void VirtualInputDevice::terminate() {
    emit terminateSignal();
}

void VirtualInputDevice::on_userInput(double x, double y) {
    axisCallback(0, -x);
    axisCallback(1, y);
}

void VirtualInputDevice::on_activate() {
    this->show();
    running = true;
}

void VirtualInputDevice::on_deactivate() {
    this->hide();
    running = false;
}

void VirtualInputDevice::on_terminate() {
    //delete ui;
    //this->close();
    QApplication::quit();
}

bool VirtualInputDevice::eventFilter(QObject *target, QEvent*event) {
    if ( event->type() == QEvent::KeyPress ) {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
        if ( !keyEvent->isAutoRepeat() ) {
            buttonCallback(keyEvent->key(), 1);
            ui->pressedButton->setText("Button: "+ QString::number(keyEvent->key()));
        }
        return true;
    }
    if ( event->type() == QEvent::KeyRelease ) {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
        if ( !keyEvent->isAutoRepeat() ) {
            buttonCallback(keyEvent->key(), 0);
            ui->pressedButton->setText("Button: ");
        }
        return true;
    }
    return false;
}

void VirtualInputDevice::resizeEvent(QResizeEvent* event) {
    static int xOffset{10};
    static int yOffset{10};
    ui->quickWidget->setGeometry(xOffset, yOffset,
        event->size().width()-2*xOffset, event->size().height()-2*yOffset - ui->pressedButton->height());
    ui->pressedButton->move(0, event->size().height()-ui->pressedButton->height());
}
