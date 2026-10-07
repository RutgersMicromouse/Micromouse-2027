#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "hardware/imu.h"

// =============================================================
// IMU HARDWARE CLASS TEST
// =============================================================

// Do NOT name this variable "imu".
// The Adafruit library already defines a namespace called "imu".
IMU testImu;

// =============================================================
// TIMING
// =============================================================

unsigned long last_update_us = 0;
unsigned long last_print_ms = 0;

const unsigned long PRINT_INTERVAL_MS = 100;

// =============================================================
// SETUP
// =============================================================

void setup()
{

  Serial.begin(115200);

  delay(3000);

  Serial.println();
  Serial.println("================================");
  Serial.println("IMU HARDWARE CLASS TEST");
  Serial.println("================================");

  // ---------------------------------------------------------
  // ENABLE ROBOT POWER
  // ---------------------------------------------------------

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  delay(100);

  // ---------------------------------------------------------
  // START I2C
  // ---------------------------------------------------------

  Wire.begin(
      PIN_I2C_SDA,
      PIN_I2C_SCL);

  Wire.setClock(400000);

  Serial.println("[TEST] I2C started.");

  // ---------------------------------------------------------
  // START IMU
  // ---------------------------------------------------------

  testImu.begin();

  // ---------------------------------------------------------
  // CHECK CONNECTION
  // ---------------------------------------------------------

  if (testImu.isHardwareConnected())
  {

    Serial.println(
        "[TEST] BNO055 connected successfully.");
  }

  else
  {

    Serial.println(
        "[TEST] ERROR: BNO055 not detected.");
  }

  // ---------------------------------------------------------
  // START TIMER
  // ---------------------------------------------------------

  last_update_us = micros();

  Serial.println();
  Serial.println("Move the mouse and watch the heading:");
  Serial.println("LEFT  -> positive");
  Serial.println("RIGHT -> negative");
  Serial.println();
}

// =============================================================
// LOOP
// =============================================================

void loop()
{

  // ---------------------------------------------------------
  // CALCULATE DT
  // ---------------------------------------------------------

  unsigned long now_us = micros();

  float dt =
      (now_us - last_update_us) /
      1000000.0f;

  last_update_us = now_us;

  // ---------------------------------------------------------
  // UPDATE REAL IMU CLASS
  // ---------------------------------------------------------

  testImu.update(
      dt,
      0.0f,
      0.0f);

  // ---------------------------------------------------------
  // PRINT AT 10 Hz
  // ---------------------------------------------------------

  unsigned long now_ms = millis();

  if (
      now_ms - last_print_ms >=
      PRINT_INTERVAL_MS)
  {

    last_print_ms = now_ms;

    // -----------------------------------------------------
    // GET VALUES FROM hardware/imu.cpp
    // -----------------------------------------------------

    float heading =
        testImu.getHeadingDeg();

    float gyro_z =
        testImu.getGyroZ();

    // -----------------------------------------------------
    // PRINT VALUES
    // -----------------------------------------------------

    Serial.printf(
        "Heading: %7.2f deg | Gyro Z: %7.2f deg/s\n",
        heading,
        gyro_z);
  }
}