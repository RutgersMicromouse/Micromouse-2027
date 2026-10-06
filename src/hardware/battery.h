#pragma once

#include <Arduino.h>

// =============================================================
// BATTERY MONITOR
// =============================================================

class BatteryMonitor {

public:

    BatteryMonitor(
        int sensePin,
        float dividerRatio
    );

    // Configure the ADC pin
    void begin();

    // Read actual battery voltage
    float readVoltage();

private:

    int sensePin_;
    float dividerRatio_;

    static const int NUM_SAMPLES = 20;
};