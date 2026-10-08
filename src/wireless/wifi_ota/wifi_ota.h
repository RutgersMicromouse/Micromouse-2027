#pragma once

// Wi-Fi hotspot, the phone app, over-the-air firmware upload, Telnet console

#include "config.h"

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
