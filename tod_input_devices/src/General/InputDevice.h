#pragma once
#include <functional>
#include <stdlib.h>
#include <string>

#define MIN_JOY_RANGE -1.0
#define MAX_JOY_RANGE 1.0

// 
class InputDevice{
public:
    explicit InputDevice(std::function<void(const int, const double)> axisCb,
        std::function<void(const int, const int)> buttonCb);
    explicit InputDevice(std::function<void(const int, const double)> axisCb,
        std::function<void(const int, const int)> buttonCb,
        std::function<void(const std::string&)> errorCb);
    virtual ~InputDevice() = default;

    void setAxisCallback(std::function<void(const int, const double)> f);
    void setButtonCallback(std::function<void(const int, const int)> f);
    void set_correction(const std::string& calibration);
    int get_number_of_axes();
    int get_number_of_buttons();
    virtual bool activate() = 0;
    virtual bool deactivate() = 0;
    virtual void terminate();
    bool running{false};

protected:
    int _number_of_buttons{-1};
    int _number_of_axes{-1};
    std::function<void(const int, const double)> axisCallback;
    std::function<void(const int, const int)> buttonCallback;
    std::function<void(const std::string&)> errorCallback;
    std::string _correction{""};
    double scaleValue(int n_value, int n_min_input, int n_max_input,
        double d_min_output = MIN_JOY_RANGE, double d_max_output = MAX_JOY_RANGE);
};
