#include <Arduino.h>

#include "config.h"
#include "wireless.h"

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("=============================================");
    Serial.println(" WIFI TEST");
    Serial.println("=============================================");

    WifiOTA::begin();

    Serial.println("WiFi started.");
}

void loop() {

    // Keep WiFi/Telnet running
    WifiOTA::handle();

    // Check for commands sent from laptop
    if (WifiOTA::hasCommand()) {

        String cmd = WifiOTA::readCommand();

        Serial.print("[WIFI] Received: ");
        Serial.println(cmd);

        WifiOTA::print("ECHO: ");
        WifiOTA::println(cmd.c_str());
    }

    delay(10);
}