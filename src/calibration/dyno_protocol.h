#pragma once

#include <Arduino.h>
#include <functional>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "../hardware/motors.h"
#include "../control/motion_controller.h"

#ifndef ENABLE_DYNO_CALIBRATION
#define ENABLE_DYNO_CALIBRATION 0
#endif

namespace DynoProtocol {
#if ENABLE_DYNO_CALIBRATION
    // Process dyno calibration commands. Returns true if handled.
    bool handleCommand(const String& cmd, Motors& motors, MotionController& motion,
                       QueueHandle_t motion_queue, std::function<void(const char*)> reply);
#else
    inline bool handleCommand(const String&, Motors&, MotionController&,
                              QueueHandle_t, std::function<void(const char*)>) {
        return false; // Zero overhead when calibration build flag is disabled
    }
#endif
}
