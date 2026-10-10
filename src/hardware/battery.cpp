#include "battery.h"

BatteryMonitor battery;

BatteryMonitor::BatteryMonitor()
    : filtered_voltage_(3.7f),
      warning_counter_(0) {
}

void BatteryMonitor::begin() {
    pinMode(PIN_BAT_SENSE, INPUT);
    // Take 10 rapid samples to seed the filter
    float sum = 0.0f;
    for (int i = 0; i < 10; ++i) {
        float raw_adc = (float)analogRead(PIN_BAT_SENSE);
        float v = (raw_adc / ADC_RESOLUTION) * ADC_REF_VOLTAGE * BATTERY_DIVIDER_RATIO;
        sum += v;
        delay(2);
    }
    filtered_voltage_ = sum / 10.0f;
    if (isMotorSwitchOn()) {
        Serial.printf("[BATTERY] Motor Battery (SW2 ON): %4.2f V (Status: %s)\n",
                      filtered_voltage_, isLow() ? "LOW WARNING!" : "HEALTHY");
    } else {
        Serial.println("[BATTERY] Motor Switch SW2 is OFF (Motors unpowered, safe for programming).");
    }
}

void BatteryMonitor::update() {
    float raw_adc = (float)analogRead(PIN_BAT_SENSE);
    float inst_v = (raw_adc / ADC_RESOLUTION) * ADC_REF_VOLTAGE * BATTERY_DIVIDER_RATIO;

    // Low-pass filter (time constant ~ 0.5s at 50Hz)
    filtered_voltage_ = 0.95f * filtered_voltage_ + 0.05f * inst_v;

    if (isCritical()) {
        warning_counter_++;
        if (warning_counter_ % 50 == 0) {
            Serial.printf("[BATTERY] CRITICAL 1S LiPo ALERT: %4.2f V! Recharge immediately.\n", filtered_voltage_);
        }
    }
}

float BatteryMonitor::getVoltage() const {
    return filtered_voltage_;
}

bool BatteryMonitor::isMotorSwitchOn() const {
    return filtered_voltage_ > BATTERY_DISCONNECTED_V;
}

bool BatteryMonitor::isLow() const {
    return isMotorSwitchOn() && (filtered_voltage_ < BATTERY_WARN_VOLTAGE);
}

bool BatteryMonitor::isCritical() const {
    return isMotorSwitchOn() && (filtered_voltage_ < BATTERY_CRITICAL_V);
}
