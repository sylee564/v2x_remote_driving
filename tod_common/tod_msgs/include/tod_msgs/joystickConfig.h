#pragma once
//joystick remapping
namespace joystick {
enum ButtonPos {
    INDICATOR_LEFT  = 0,
    INDICATOR_RIGHT = 1,
    EMERGENCY_SIGNAL = 2,
    INCREASE_SPEED  = 3,
    DECREASE_SPEED  = 4,
    DRIVE      = 5,
    REVERSE    = 6,
    NEUTRAL  = 7,
    PARK  = 8,
    REMOTE = 9,
    VIDEO = 10,
    AVM = 11,
    AEB = 12,
    PARK_MODE = 13,
    OBJECT = 14,
    CRUISE = 15
};

enum AxesPos {
    STEERING = 0,
    THROTTLE = 1,
    BRAKE = 2
};
} // namespace joystick