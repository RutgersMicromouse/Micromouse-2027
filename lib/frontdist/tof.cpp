#include "tof.h"
#include <Wire.h>
#include <VL53L1X.h>

#define PIN_FRONT_XSHUT 2
#define PIN_LEFT_XSHUT  4
#define PIN_RIGHT_XSHUT 3

VL53L1X tofFront;
VL53L1X tofLeft;
VL53L1X tofRight;

int16_t frontLast = -1;
int16_t leftLast  = -1;
int16_t rightLast = -1;

struct SensorDebugState {
    uint32_t lastSampleMs = 0;
    uint32_t lastNoDataLogMs = 0;
    uint32_t samples = 0;
    uint32_t timeouts = 0;
};

SensorDebugState frontDebug;
SensorDebugState leftDebug;
SensorDebugState rightDebug;
uint8_t tofSetupCount = 0;

static void debugLine(const String &message) {
    Serial.println(message);
    Serial1.println(message);
}

static void initSensor(VL53L1X &sensor, uint8_t address, const char* name) {
    debugLine(String("TOF ") + name + ": begin init, default address=0x29, target address=0x" + String(address, HEX));
    sensor.setBus(&Wire);
    sensor.setTimeout(500);

    uint8_t modelId = sensor.readReg(VL53L1X::IDENTIFICATION__MODEL_ID);
    debugLine(String("TOF ") + name + ": model ID=0x" + String(modelId, HEX) +
              ", I2C status=" + String(sensor.last_status));

    bool ok = sensor.init();
    debugLine(String("TOF ") + name + ": init=" + String(ok ? "OK" : "FAILED") +
              ", I2C status=" + String(sensor.last_status));

    if (ok) {
        sensor.setAddress(address);
        debugLine(String("TOF ") + name + ": assigned address=0x" + String(sensor.getAddress(), HEX) +
                  ", I2C status=" + String(sensor.last_status));

        bool modeOk = sensor.setDistanceMode(VL53L1X::Short);
        bool timingOk = sensor.setMeasurementTimingBudget(20000);
        debugLine(String("TOF ") + name + ": short mode=" + String(modeOk ? "OK" : "FAILED") +
                  ", timing budget=20000us " + String(timingOk ? "OK" : "FAILED") +
                  ", I2C status=" + String(sensor.last_status));

        sensor.startContinuous(25);
        debugLine(String("TOF ") + name + ": continuous ranging started, period=25ms");
    } else {
        debugLine(String("TOF ") + name + ": initialization failed; check power, XSHUT, SDA/SCL, and I2C address");
    }
}

void tofSetup() {
    ++tofSetupCount;
    debugLine(String("TOF SETUP BEGIN #") + String(tofSetupCount));

    pinMode(PIN_FRONT_XSHUT, OUTPUT);
    pinMode(PIN_LEFT_XSHUT, OUTPUT);
    pinMode(PIN_RIGHT_XSHUT, OUTPUT);

    digitalWrite(PIN_FRONT_XSHUT, LOW);
    digitalWrite(PIN_LEFT_XSHUT, LOW);
    digitalWrite(PIN_RIGHT_XSHUT, LOW);
    delay(100);
    debugLine("TOF: all XSHUT pins LOW");

    digitalWrite(PIN_FRONT_XSHUT, HIGH);
    delay(100);
    debugLine("TOF: Front XSHUT HIGH");
    initSensor(tofFront, 0x30, "Front");

    digitalWrite(PIN_LEFT_XSHUT, HIGH);
    delay(100);
    debugLine("TOF: Left XSHUT HIGH");
    initSensor(tofLeft, 0x32, "Left");

    digitalWrite(PIN_RIGHT_XSHUT, HIGH);
    delay(100);
    debugLine("TOF: Right XSHUT HIGH");
    initSensor(tofRight, 0x34, "Right");

    uint32_t now = millis();
    frontDebug.lastSampleMs = now;
    leftDebug.lastSampleMs = now;
    rightDebug.lastSampleMs = now;
    debugLine(String("TOF SETUP END #") + String(tofSetupCount));
}

int16_t readSensor(VL53L1X &sensor, int16_t &last, SensorDebugState &debugState, const char* name) {
    while (sensor.dataReady()) {
        uint16_t reading = sensor.read(false);
        if (sensor.timeoutOccurred()) {
            ++debugState.timeouts;
            if (debugState.timeouts == 1 || debugState.timeouts % 25 == 0) {
                debugLine(String("TOF ") + name + ": read timeout #" + String(debugState.timeouts) +
                          ", I2C status=" + String(sensor.last_status));
            }
            break;
        }
        last = static_cast<int16_t>(reading);
        ++debugState.samples;
        debugState.lastSampleMs = millis();
        if (debugState.samples == 1 || debugState.samples % 100 == 0) {
            debugLine(String("TOF ") + name + ": sample=" + String(last) + "mm, count=" +
                      String(debugState.samples) + ", range status=" +
                      String(VL53L1X::rangeStatusToString(sensor.ranging_data.range_status)) +
                      ", I2C status=" + String(sensor.last_status));
        }
    }

    uint32_t now = millis();
    if (now - debugState.lastSampleMs > 1000 && now - debugState.lastNoDataLogMs > 2000) {
        debugState.lastNoDataLogMs = now;
        debugLine(String("TOF ") + name + ": no new sample for " +
                  String(now - debugState.lastSampleMs) + "ms, returning last=" + String(last) +
                  "mm, I2C status=" + String(sensor.last_status));
    }

    return last;
}

int16_t front() { return readSensor(tofFront, frontLast, frontDebug, "Front"); }
int16_t left()  { return readSensor(tofLeft,  leftLast,  leftDebug,  "Left"); }
int16_t right() { return readSensor(tofRight, rightLast, rightDebug, "Right"); }
