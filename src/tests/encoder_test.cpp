#include <Arduino.h>

#include "config.h"
#include "hardware/encoders.h"

Encoders encoders;

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("==============================");
    Serial.println(" ENCODER TEST");
    Serial.println("==============================");

    // Enable board power
    pinMode(PIN_MOTOR_BAT_CTRL, OUTPUT);
    digitalWrite(PIN_MOTOR_BAT_CTRL, HIGH);

    delay(300);

    // Initialize production encoder code
    encoders.begin();

    Serial.println("Encoders initialized.");
    Serial.println("Manually rotate the wheels.");
    Serial.println();
}

void loop() {

    static unsigned long lastUpdate = micros();
    static unsigned long lastPrint = millis();

    unsigned long now = micros();

    float dt = (now - lastUpdate) / 1000000.0f;
    lastUpdate = now;

    // Uses hardware/encoders.cpp
    encoders.update(dt);

    EncoderState enc = encoders.getState();

    if (millis() - lastPrint >= 100) {

        lastPrint = millis();

        Serial.printf(
            "LEFT: %ld ticks | RIGHT: %ld ticks\n",
            (long)enc.left_ticks_total,
            (long)enc.right_ticks_total
        );
    }

    delay(2);
}