#pragma once

#include <Arduino.h>
#include "config.h"

// =============================================================================
// Battery Voltage Sensing
// Pin: 21 (A7)
// Divider: R1=100k to BAT+, R2=33k to GND
// =============================================================================

class BatteryMonitor {
public:
    BatteryMonitor();

    void begin();
    void update();

    float getVoltage() const;
    bool isLow() const;
    bool isCritical() const;

private:
    float filtered_voltage_;
    uint8_t warning_counter_;
};

extern BatteryMonitor battery;
