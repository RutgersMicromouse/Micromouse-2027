#pragma once

// The robot's shared objects. They are created once in main.cpp; every other file reaches them
// through this header.
//
// Who may touch what:
//   Core 1 (motion task, 500 Hz) owns the encoders, IR sensors, IMU, motors, and motion controller.
//   Core 0 (navigation task) owns the navigator and talks to Core 1 only through
//   g_motion_cmd_queue, the telemetry snapshot, and the request functions below.

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "config.h"
#include "types.h"

#include "hardware.h"
#include "control.h"
#include "navigation.h"

extern Encoders         g_encoders;
extern Motors           g_motors;
extern IRSensors        g_ir_sensors;
extern IMU              g_imu;
extern MotionController g_motion_controller;
extern Navigator*       g_navigator;

// Motion commands travelling from Core 0 to Core 1
extern QueueHandle_t    g_motion_cmd_queue;

// Copies the latest sensor snapshot published by the motion task (updated at 50 Hz).
// Returns false if it could not be read in time; `out` is then left unchanged.
bool getTelemetry(RobotTelemetry& out);

// The 5 Hz status line on USB / Bluetooth / Telnet (off at power-on)
void setTelemetryStreaming(bool on);
bool isTelemetryStreaming();

// Ask the motion task to zero the encoders / heading. Carried out between motions.
void requestEncoderReset();
void requestHeadingReset();

// Call before starting any run. The robot has just been put down by hand, so wherever it is
// pointing now becomes "straight ahead" and its distance counters start from zero. Returns once
// the motion task has done it.
void prepareForNewRun();
