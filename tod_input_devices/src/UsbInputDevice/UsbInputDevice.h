#pragma once
#include <fcntl.h>
#include <iostream>
#include <unistd.h>
#include <linux/joystick.h>

#include <thread>
#include <cstring>
#include <string>
#include "InputDevice.h"
#include "Correction.h"

#define MIN_POSITION_INCREMENTS_USB -32767
#define MAX_POSITION_INCREMENTS_USB 32767

class UsbInputDevice : public InputDevice{
public:
    UsbInputDevice(std::function<void(const int, const double)> axisCb,
        std::function<void(const int, const int)> buttonCb,
        std::function<void(const std::string&)> errorCb);
    ~UsbInputDevice();

    bool activate() override;
    bool deactivate() override;

private:
    struct js_event _jsevent;
    const char *_device;
    int _js;
    std::thread _readUsbThread;

    void init_size();
    int read_event(int fd, struct js_event *event);
    void run();
};