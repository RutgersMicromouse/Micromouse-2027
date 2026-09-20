#pragma once

#include <Arduino.h>
#include "config.h"

#if ENABLE_BLE_DEBUG

class BLEDebug {
public:
    static void begin(const char* device_name = "Antigrav-Mouse");
    static bool isConnected();
    static void print(const char* str);
    static void println(const char* str);
    static void printf(const char* fmt, ...);

    // Command interface from remote phone / PC
    static bool hasCommand();
    static String readCommand();
};

#endif // ENABLE_BLE_DEBUG
