#pragma once

#include <Arduino.h>
#include "config.h"

#if ENABLE_BLE_DEBUG

class BLEDebug {
public:

    // Start BLE
    static void begin(
        const char* device_name = "Antigrav-Mouse"
    );

    // Connection status
    static bool isConnected();


    // =========================================================
    // Existing UART Debug Interface
    // =========================================================

    static void print(const char* str);

    static void println(const char* str);

    static void printf(
        const char* fmt,
        ...
    );


    // =========================================================
    // Commands From Phone
    // =========================================================

    static bool hasCommand();

    static String readCommand();


    // =========================================================
    // Live Telemetry
    // =========================================================

    static void updateTelemetry(
        float battery_voltage,
        float heading_deg,
        long left_encoder,
        long right_encoder
    );
};

#endif // ENABLE_BLE_DEBUG