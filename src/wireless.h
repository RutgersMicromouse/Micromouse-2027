#pragma once

// Wireless links (both can be switched off in config.h for competition):
//   BLEDebug - Bluetooth text console + readable telemetry for the web dashboard
//   WifiOTA  - Wi-Fi hotspot, the phone app, over-the-air firmware upload, Telnet console

#include "config.h"

// ==============================================================================
// BLUETOOTH LOW ENERGY
// ==============================================================================

#include <Arduino.h>

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

// ==============================================================================
// WI-FI OTA & TELNET
// ==============================================================================

#include <Arduino.h>

#if ENABLE_WIFI_OTA

class WifiOTA {
public:
    static void begin();
    static void handle();
    static bool isClientConnected();

    // Wireless Telnet & Web Serial streaming
    static void print(const char* str);
    static void println(const char* str);
    static void printf(const char* fmt, ...);

    // Incoming wireless commands from Telnet or from the phone app
    static bool hasCommand();
    static String readCommand();
    static void commandDone();   // Call when that command has been carried out (the app shows its button as finished)

    // The phone app (http://192.168.4.1). `provider` returns the robot's live numbers as a JSON
    // object; appendWebLog feeds it the same text the serial monitor shows.
    static void setStatusProvider(String (*provider)());
    static void appendWebLog(const char* text, size_t length);

    static IPAddress getIP();
};

#endif // ENABLE_WIFI_OTA
