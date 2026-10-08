#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "wireless/ble_debug/ble_debug.h"
#include "battery.h"
#include "hardware/encoders/encoders.h"
#include "hardware/motors/motors.h"
#include "hardware/ir_sensors/ir_sensors.h"
#include "hardware/imu/imu.h"


// =============================================================
// BLE TELEMETRY TEST
//
// Battery
// Heading
// Left Encoder
// Right Encoder
// =============================================================


// =============================================================
// HARDWARE
// =============================================================

BatteryMonitor battery(
    PIN_VSENSE_COM,
    BATTERY_DIVIDER_RATIO
);


IMU imu;


Encoders encoders;


// =============================================================
// STATE
// =============================================================

bool was_connected = false;


unsigned long last_update_us = 0;


// Update BLE characteristic values every 100 ms
unsigned long last_telemetry_update_ms = 0;


const unsigned long TELEMETRY_UPDATE_MS = 100;


// =============================================================
// SETUP
// =============================================================

void setup() {

    Serial.begin(
        115200
    );


    delay(
        2000
    );


    Serial.println();

    Serial.println(
        "============================================="
    );

    Serial.println(
        " BLE ROBOT TELEMETRY TEST"
    );

    Serial.println(
        "============================================="
    );


    // =========================================================
    // BATTERY
    // =========================================================

    battery.begin();


    // =========================================================
    // I2C
    // =========================================================

    Wire.begin(
        PIN_I2C_SDA,
        PIN_I2C_SCL
    );


    Wire.setClock(
        I2C_CLOCK_SPEED
    );


    // =========================================================
    // IMU
    // =========================================================

    Serial.println(
        "Starting IMU..."
    );


    imu.begin();


    if (
        imu.isHardwareConnected()
    ) {

        Serial.println(
            "[IMU] BNO055 detected"
        );

    } else {

        Serial.println(
            "[IMU] BNO055 NOT detected"
        );
    }


    // =========================================================
    // ENCODERS
    // =========================================================

    Serial.println(
        "Starting encoders..."
    );


    encoders.begin();


    // =========================================================
    // BLUETOOTH
    // =========================================================

    BLEDebug::begin(
        BLE_DEVICE_NAME
    );


    Serial.printf(
        "Advertising as '%s'\n",
        BLE_DEVICE_NAME
    );


    Serial.println(
        "Waiting for phone..."
    );


    // =========================================================
    // START TIMING
    // =========================================================

    last_update_us =
        micros();
}


// =============================================================
// LOOP
// =============================================================

void loop() {

    // =========================================================
    // CALCULATE LOOP TIME
    // =========================================================

    unsigned long now_us =
        micros();


    float dt =
        (
            now_us
            -
            last_update_us
        )
        /
        1000000.0f;


    last_update_us =
        now_us;


    // =========================================================
    // UPDATE ENCODERS
    // =========================================================

    encoders.update(
        dt
    );


    EncoderState enc =
        encoders.getState();


    // =========================================================
    // CALCULATE ENCODER YAW RATE
    // =========================================================

    float encoder_yaw_rate_rad_s =
        (
            enc.right_speed_mm_s
            -
            enc.left_speed_mm_s
        )
        /
        WHEEL_BASE_MM;


    float encoder_yaw_rate_deg_s =
        encoder_yaw_rate_rad_s
        *
        (
            180.0f
            /
            PI
        );


    // =========================================================
    // UPDATE IMU
    // =========================================================

    imu.update(
        dt,
        encoder_yaw_rate_deg_s,
        enc.linear_speed_mm_s
    );


    // =========================================================
    // CHECK BLE CONNECTION
    // =========================================================

    bool connected =
        BLEDebug::isConnected();


    if (
        connected
        !=
        was_connected
    ) {

        was_connected =
            connected;


        if (
            connected
        ) {

            Serial.println(
                "[BLE] Phone CONNECTED"
            );

        } else {

            Serial.println(
                "[BLE] Phone DISCONNECTED"
            );
        }
    }


    // =========================================================
    // UPDATE TELEMETRY CHARACTERISTICS
    // =========================================================

    if (
        millis()
        -
        last_telemetry_update_ms
        >=
        TELEMETRY_UPDATE_MS
    ) {

        last_telemetry_update_ms =
            millis();


        float voltage =
            battery.readVoltage();


        float heading =
            imu.getHeadingDeg();


        long left_ticks =
            (long)
            enc.left_ticks_total;


        long right_ticks =
            (long)
            enc.right_ticks_total;


        // -----------------------------------------------------
        // Update the CURRENT value of each BLE characteristic.
        //
        // This does NOT send UART notifications.
        // -----------------------------------------------------

        BLEDebug::updateTelemetry(
            voltage,
            heading,
            left_ticks,
            right_ticks
        );


        // USB serial is still useful while connected to laptop
        Serial.printf(
            "B:%.2fV H:%.1f L:%ld R:%ld\n",
            voltage,
            heading,
            left_ticks,
            right_ticks
        );
    }


    // =========================================================
    // COMMANDS FROM PHONE
    // =========================================================

    if (
        BLEDebug::hasCommand()
    ) {

        String cmd =
            BLEDebug::readCommand();


        // -----------------------------------------------------
        // Reset Encoders
        // -----------------------------------------------------

        if (
            cmd.equalsIgnoreCase(
                "resetenc"
            )
        ) {

            encoders.reset();


            BLEDebug::println(
                "Encoders reset"
            );
        }


        // -----------------------------------------------------
        // Reset Heading
        // -----------------------------------------------------

        else if (
            cmd.equalsIgnoreCase(
                "resetheading"
            )
        ) {

            imu.resetHeading(
                0.0f
            );


            BLEDebug::println(
                "Heading reset"
            );
        }


        // -----------------------------------------------------
        // Reset Everything
        // -----------------------------------------------------

        else if (
            cmd.equalsIgnoreCase(
                "reset"
            )
        ) {

            encoders.reset();


            imu.resetHeading(
                0.0f
            );


            BLEDebug::println(
                "Heading + encoders reset"
            );
        }


        // -----------------------------------------------------
        // Help
        // -----------------------------------------------------

        else if (
            cmd.equalsIgnoreCase(
                "help"
            )
        ) {

            BLEDebug::println(
                "Commands: reset, resetenc, resetheading, help"
            );
        }


        // -----------------------------------------------------
        // Unknown
        // -----------------------------------------------------

        else {

            BLEDebug::printf(
                "Unknown command: %s\r\n",
                cmd.c_str()
            );
        }
    }


    delay(
        2
    );
}