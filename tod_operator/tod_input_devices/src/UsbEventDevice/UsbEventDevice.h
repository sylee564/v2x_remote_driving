#pragma once
#include <linux/input.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
#include "bitmaskros.h"

#define DEFAULT_AXIS_INDEX          0
#define DEFAULT_AXIS_CODE       ABS_X

class UsbEventDevice {
public:
    explicit UsbEventDevice(const std::string &device_namespace);
    ~UsbEventDevice() { if (ok()) deleteEffect(); }
    bool ok() const { return _ok; }
    void set_force_feedback(const double ff_value);
    void reset();

private:
    bool _ok{false};
    const std::vector<std::string> axis_names{ "Steering_Wheel", "Accelerator", "Brake"};
    const std::vector<int> axis_codes{ ABS_X, ABS_Z, ABS_RZ };

    int _axis_index{DEFAULT_AXIS_INDEX};
    int _axis_code{DEFAULT_AXIS_CODE};
    // int _button_code{DEFAULT_KEY};
    bool _autocenter_off;
    int _device_handle;
    int _axis_min, _axis_max;

    struct ff_effect _effect;

    bool openDevice(const std::string &deviceNamespace);
    void initializeDevice();
    void createEvent(const double force);
    void deleteEffect();
};
