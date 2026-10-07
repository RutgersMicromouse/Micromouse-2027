// bench/battery_test.cpp

#include <Arduino.h>
#include "battery.h"

const int PIN_VSENSE_COM = 12;       // pin is gpio12
const float BATTERY_DIVIDER_RATIO = 2.0f;

BatteryMonitor battery(
    PIN_VSENSE_COM,
    BATTERY_DIVIDER_RATIO
);

void setup() {

    Serial.begin(115200);
    delay(2000);

    battery.begin();

    Serial.println("=== BATTERY TEST ===");
}

void loop() {

    float voltage = battery.readVoltage();

    Serial.print("Battery Voltage: ");
    Serial.print(voltage, 3);
    Serial.println(" V");

    delay(500);
}