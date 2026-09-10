#include "tof.h"
#include <Wire.h>
#include <VL53L4CD.h>

#define PIN_FRONT_XSHUT 2
#define PIN_LEFT_XSHUT  4
#define PIN_RIGHT_XSHUT 3

VL53L4CD tofFront;
VL53L4CD tofLeft;
VL53L4CD tofRight;

int16_t frontLast = -1;
int16_t leftLast  = -1;
int16_t rightLast = -1;

static void initSensor(VL53L4CD &sensor, uint8_t address, const char* name) {
    sensor.setBus(&Wire);
    sensor.setTimeout(500);

    // Read model ID registers before init
    uint8_t id_high = sensor.readReg(0x010F);
    uint8_t id_low  = sensor.readReg(0x0110);
    Serial.print(name); Serial.print(" Model ID: 0x");
    Serial.print(id_high, HEX);
    Serial.println(id_low, HEX);

    bool ok = sensor.init();
    Serial.print(name); Serial.print(" init: "); Serial.println(ok);

    if (ok) {
        sensor.setAddress(address);
        sensor.startContinuous();
        Serial.print(name); Serial.println(" OK");
    } else {
        Serial.print(name); Serial.print(" FAILED, last_status: ");
        Serial.println(sensor.last_status);
    }

    Serial.println("TOF SETUP DONE");
}

void tofSetup() {
    pinMode(PIN_FRONT_XSHUT, OUTPUT);
    pinMode(PIN_LEFT_XSHUT, OUTPUT);
    pinMode(PIN_RIGHT_XSHUT, OUTPUT);

    digitalWrite(PIN_FRONT_XSHUT, LOW);
    digitalWrite(PIN_LEFT_XSHUT, LOW);
    digitalWrite(PIN_RIGHT_XSHUT, LOW);
    delay(100);

    digitalWrite(PIN_FRONT_XSHUT, HIGH);
    delay(100);
    initSensor(tofFront, 0x30, "Front");

    digitalWrite(PIN_LEFT_XSHUT, HIGH);
    delay(100);
    initSensor(tofLeft, 0x32, "Left");

    digitalWrite(PIN_RIGHT_XSHUT, HIGH);
    delay(100);
    initSensor(tofRight, 0x34, "Right");
}

int16_t readSensor(VL53L4CD &sensor, int16_t &last) {
    while (sensor.dataReady()) {
        last = (int16_t)sensor.read(true);
    }
    return last;
}

int16_t front() { return readSensor(tofFront, frontLast); }
int16_t left()  { return readSensor(tofLeft,  leftLast);  }
int16_t right() { return readSensor(tofRight, rightLast); }
