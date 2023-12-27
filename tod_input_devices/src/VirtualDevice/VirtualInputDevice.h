#pragma once
#include <QMainWindow>
#include "InputDevice.h"
namespace Ui {
class JoystickWindow;
}

class VirtualInputDevice : public QMainWindow, public InputDevice {
    Q_OBJECT

public:
    explicit VirtualInputDevice(std::function<void(const int, const double)> axisCb,
        std::function<void(const int, const int)> buttonCb, QWidget *parent = 0);
    ~VirtualInputDevice();

    bool activate() override;
    bool deactivate() override;
    void terminate() override;

private slots:
    void on_userInput(double x, double y);
    void on_activate();
    void on_deactivate();
    void on_terminate();

signals:
    void activateSignal();
    void deactivateSignal();
    void terminateSignal();


private:
    Ui::JoystickWindow *ui;
    bool eventFilter(QObject *target, QEvent *e);
    void resizeEvent(QResizeEvent *event);
};
