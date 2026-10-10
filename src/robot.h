#pragma once

// The robot's shared objects. They are created once in main.cpp; every other file reaches them
// through this header.
//
// Who may touch what:
//   Core 1 (motion task, 500 Hz) owns the encoders, IR sensors, IMU, motors, and motion controller.
//   Core 0 (navigation task) steps the navigator and talks to Core 1 only through
//   g_motion_cmd_queue, the "move finished" queue in main.cpp, and the request functions below.
//   Core 0 (operator task: console, phone-app buttons, hand waves) may start and stop runs, so
//   every call that CHANGES the navigator is made while holding a NavigatorLock.

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "config.h"
#include "types.h"

#include "hardware/encoders/encoders.h"
#include "hardware/motors/motors.h"
#include "hardware/ir_sensors/ir_sensors.h"
#include "hardware/imu/imu.h"
#include "control/motion_controller/motion_controller.h"
#include "navigation/navigator/navigator.h"

extern Encoders         g_encoders;
extern Motors           g_motors;
extern IRSensors        g_ir_sensors;
extern IMU              g_imu;
extern MotionController g_motion_controller;
extern Navigator*       g_navigator;

// Motion commands travelling from Core 0 to Core 1
extern QueueHandle_t    g_motion_cmd_queue;

// Held by whichever task is changing the navigator (stepping it, starting or stopping a run).
// Create one as a local variable; it lets go at the end of the block:
//     { NavigatorLock lock; g_navigator->stop(); }
// Keep the block short: the navigation task waits for it before it can send the next move.
extern SemaphoreHandle_t g_navigator_mutex;
struct NavigatorLock {
    NavigatorLock()  { xSemaphoreTake(g_navigator_mutex, portMAX_DELAY); }
    ~NavigatorLock() { xSemaphoreGive(g_navigator_mutex); }
    NavigatorLock(const NavigatorLock&) = delete;
    NavigatorLock& operator=(const NavigatorLock&) = delete;
};

// Copies the latest sensor snapshot published by the motion task (updated at 50 Hz).
// Returns false if it could not be read in time; `out` is then left unchanged.
bool getTelemetry(RobotTelemetry& out);

// The 5 Hz status line on USB / Bluetooth / Telnet (off at power-on)
void setTelemetryStreaming(bool on);
bool isTelemetryStreaming();

// STOP, from anywhere (app, Bluetooth, USB, the console). Safe to call from any task or callback.
// The motion task brakes on its very next tick (2 ms) and throws away any move waiting in its
// queue, and keeps doing so until the navigation task has stopped the run. It does not wait for
// the console, so it cannot be held up or dropped behind another command.
void requestStop();
uint32_t stopCount();     // How many STOPs there have been since power-on (to notice one that came and went)

// Ask the motion task to zero the encoders / heading. Carried out between motions.
void requestEncoderReset();
void requestHeadingReset();

// Call before starting any run. The robot has just been put down by hand, so wherever it is
// pointing now becomes "straight ahead" and its distance counters start from zero. Returns once
// the motion task has done it.
void prepareForNewRun();
