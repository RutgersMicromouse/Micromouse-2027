#include "imu.h"

Adafruit_BNO08x bno;
sh2_SensorValue_t sensorValue;

static double angleOffset = 0;

double readRawAngle() {
    static double lastYaw = 0;
    unsigned long start = millis();
    while (millis() - start < 50) {
        if (bno.getSensorEvent(&sensorValue)) {
            if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
                float qw = sensorValue.un.rotationVector.real;
                float qx = sensorValue.un.rotationVector.i;
                float qy = sensorValue.un.rotationVector.j;
                float qz = sensorValue.un.rotationVector.k;
                lastYaw = atan2(2.0 * (qw*qz + qx*qy),
                                1.0 - 2.0 * (qy*qy + qz*qz))
                          * (180.0 / PI);
                return lastYaw;
            }
        }
    }
    return lastYaw; // fallback if no event in 50ms
}

void imuSetup() {
    Wire.setClock(400000);
    if (!bno.begin_I2C(0x4B)) {
        Serial.println("No BNO08x detected");
        while (1);
    }
    bno.enableReport(SH2_ROTATION_VECTOR, 10000); // 100Hz
    delay(500); // let BNO stabilize
    angleOffset = readRawAngle(); // read once to set offset
    Serial.println("IMU SETUP DONE");
}

double angle() {
    double raw = readRawAngle();
    double result = raw - angleOffset;
    // normalize to -180..180
    if (result > 180)  result -= 360;
    if (result < -180) result += 360;
    return result;
}
