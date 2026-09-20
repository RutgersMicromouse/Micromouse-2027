#pragma once

#include <Arduino.h>
#include "config.h"

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

    // Incoming wireless commands from Telnet
    static bool hasCommand();
    static String readCommand();

    static IPAddress getIP();
};

#endif // ENABLE_WIFI_OTA
