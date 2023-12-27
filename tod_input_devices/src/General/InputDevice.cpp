#include "InputDevice.h"

InputDevice::InputDevice(
    std::function<void(const int, const double)> axisCb, std::function<void(const int, const int)> buttonCb) {
    axisCallback = axisCb;
    buttonCallback = buttonCb;
}
InputDevice::InputDevice(
    std::function<void(const int, const double)> axisCb, std::function<void(const int, const int)> buttonCb,
    std::function<void(const std::string&)> errorCb) {
    errorCallback = errorCb;
    axisCallback = axisCb;
    buttonCallback = buttonCb;
}

void InputDevice::setAxisCallback(std::function<void(const int, const double)> f) {
    axisCallback = f;
}
void InputDevice::setButtonCallback(std::function<void(const int, const int)> f) {
    buttonCallback = f;
}

void InputDevice::terminate() {
    deactivate();
}

void InputDevice::set_correction(const std::string& correction) {
    _correction = correction;
}

double InputDevice::scaleValue(int n_value, int n_min_input, int n_max_input, double d_min_output, double d_max_output) {
    double d_value;
    d_value = d_min_output + (double)(n_value - n_min_input)*(d_max_output - d_min_output) / (double)(n_max_input - n_min_input);
    if (d_value > d_max_output)
        return d_max_output;
    else if (d_value < d_min_output)
        return d_min_output;
    else
        return d_value;
}

int InputDevice::get_number_of_axes() {
    return _number_of_axes;
}
int InputDevice::get_number_of_buttons() {
    return _number_of_buttons;
}
