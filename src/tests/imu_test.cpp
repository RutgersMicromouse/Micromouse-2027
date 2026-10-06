


#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Pin Definitions based on PCB layout
const int POWER_ENABLE = 13; // Enables board power rails
const int SDA_PIN      = 21; // SDA Pin
const int SCL_PIN      = 20; // SCL Pin

// BNO055 Instance (ID: 55, Address: 0x28 default; try 0x29 if 0x28 fails)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup() {
  Serial.begin(115200);
  delay(3000); // Allow USB CDC connection time

  Serial.println("\n=========================================");
  Serial.println("  ESP32-S3 BNO055 IMU Diagnostic Test    ");
  Serial.println("=========================================");

  // 1. Enable Power Rails
  pinMode(POWER_ENABLE, OUTPUT);
  digitalWrite(POWER_ENABLE, HIGH);
  delay(500); // Power stabilization delay

  // 2. Initialize I2C Bus
  Wire.begin(SDA_PIN, SCL_PIN);

  // 3. Initialize BNO055
  if (!bno.begin()) {
    Serial.println("ERROR: BNO055 not detected at address 0x28!");
    Serial.println("Checking address 0x29...");

    // Retry with alternate address 0x29 (if ADR pin is high)
    bno = Adafruit_BNO055(55, 0x29, &Wire);
    if (!bno.begin()) {
      Serial.println("ERROR: BNO055 not detected at 0x29 either. Check power and I2C lines.");
      while (1) { delay(100); }
    }
  }

  Serial.println("SUCCESS: BNO055 connected successfully!");
  
  // Use external crystal for higher precision if present on module
  bno.setExtCrystalUse(true);

  delay(1000);
}

void loop() {
  // 1. Get Orientation Event (Euler Angles: Heading, Pitch, Roll)
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  // 2. Read Calibration Status (0 = Uncalibrated, 3 = Fully Calibrated)
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  // 3. Print Data
  Serial.printf("Heading: %6.2f° | Pitch: %6.2f° | Roll: %6.2f°  ||  Calib -> Sys:%d G:%d A:%d M:%d\n",
                orientationData.orientation.x,  // Yaw / Heading
                orientationData.orientation.y,  // Pitch
                orientationData.orientation.z,  // Roll
                system, gyro, accel, mag);

  delay(100); // 10 Hz refresh rate
}