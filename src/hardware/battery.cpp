#include "battery.h"

// =============================================================
// CONSTRUCTOR
// =============================================================

BatteryMonitor::BatteryMonitor(
    int sensePin,
    float dividerRatio
) {
    sensePin_ = sensePin;
    dividerRatio_ = dividerRatio;
}


// =============================================================
// INITIALIZE BATTERY MONITOR
// =============================================================

void BatteryMonitor::begin() {

    pinMode(sensePin_, INPUT);

    // ESP32 ADC resolution
    analogReadResolution(12);
}


// =============================================================
// READ BATTERY VOLTAGE
// =============================================================

float BatteryMonitor::readVoltage() {

    uint32_t totalMilliVolts = 0;


    // Take multiple ADC measurements
    for (int i = 0; i < NUM_SAMPLES; i++) {

        totalMilliVolts +=
            analogReadMilliVolts(sensePin_);

        delay(2);
    }


    // Calculate average ADC voltage
    float averageMilliVolts =
        totalMilliVolts / (float)NUM_SAMPLES;


    // Convert millivolts to volts
    float pinVoltage =
        averageMilliVolts / 1000.0f;


    // The voltage divider reduces the battery voltage
    // before it reaches the ESP32.
    //
    // Multiply by the divider ratio to reconstruct
    // the original battery voltage.
    float batteryVoltage =
        pinVoltage * dividerRatio_;


    return batteryVoltage;
}