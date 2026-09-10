#pragma once

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BNO08x.h"
#include <SPI.h>

// default I2C address 0x28

void imuSetup();
double angle();