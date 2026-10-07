#pragma once

#include <Arduino.h>

// Averaged battery voltage reader used by the bench tests (battery_test, ble_test).
// The robot firmware reads the battery inside its control loop instead (src/main.cpp).
class BatteryMonitor {
public:
    BatteryMonitor(int sensePin, float dividerRatio)
        : sensePin_(sensePin), dividerRatio_(dividerRatio) {}

    // Configure the ADC pin
    void begin() {
        pinMode(sensePin_, INPUT);
        analogReadResolution(12);
    }

    // Read actual battery voltage (averages 20 samples, takes ~40 ms)
    float readVoltage() {
        uint32_t totalMilliVolts = 0;
        for (int i = 0; i < NUM_SAMPLES; i++) {
            totalMilliVolts += analogReadMilliVolts(sensePin_);
            delay(2);
        }
        float pinVoltage = (totalMilliVolts / (float)NUM_SAMPLES) / 1000.0f;

        // The voltage divider reduces the battery voltage before it reaches the ESP32.
        // Multiply by the divider ratio to reconstruct the original battery voltage.
        return pinVoltage * dividerRatio_;
    }

private:
    int sensePin_;
    float dividerRatio_;

    static const int NUM_SAMPLES = 20;
};
