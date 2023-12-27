#pragma once
// used in SecondaryControlCmd and VehicleData messages

enum eIndicator {
    INDICATOR_OFF = 0,
    INDICATOR_LEFT = 1,
    INDICATOR_RIGHT = 2
};

enum eGearPosition {
    GEARPOSITION_PARK = 0,
    GEARPOSITION_REVERSE = 7,
    GEARPOSITION_NEUTRAL = 6,
    GEARPOSITION_DRIVE = 5
};

enum eEmergencyLight {
    EmergencyLight_OFF = 0,
    EmergencyLight_ON = 1
};

enum eWiper {
    WIPER_OFF = 0,
    WIPER_ON = 1,
    WIPER_INTERVAL = 2
};

enum eHeadLight {
    HEADLIGHT_OFF = 0,
    HEADLIGHT_ON = 1
};

enum eFlashLight {
    FLASHLIGHT_OFF = 0,
    FLASHLIGHT_ON = 1
};