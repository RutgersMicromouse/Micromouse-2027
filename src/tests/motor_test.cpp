#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "hardware/motors.h"
#include "hardware/encoders.h"


// =============================================================
// MOTOR + ENCODER HARDWARE CLASS TEST
//
// This test uses the REAL production classes:
//
//     motor_test.cpp
//          |
//          +----> Motors ----> physical motors
//          |
//          +----> Encoders --> physical encoders
//
// There is NO separate Motoron or PCNT implementation here.
// =============================================================


// =============================================================
// HARDWARE OBJECTS
// =============================================================

Motors motors;
Encoders encoders;


// =============================================================
// TEST SETTINGS
// =============================================================

// Equivalent to roughly the old Motoron speed of ~200 / 800.
const float TEST_EFFORT = 0.25f;

const unsigned long SAMPLE_INTERVAL_MS = 100;
const unsigned long TEST_DURATION_MS   = 3000;

const int NUM_SAMPLES =
    TEST_DURATION_MS / SAMPLE_INTERVAL_MS;


// =============================================================
// PRINT CURRENT ENCODER STATE
// =============================================================

void printEncoderState(
    unsigned long elapsed_ms,
    float left_effort,
    float right_effort
) {

    EncoderState enc =
        encoders.getState();


    Serial.print(elapsed_ms);
    Serial.print(" ms");


    // ---------------------------------------------------------
    // MOTOR COMMANDS
    // ---------------------------------------------------------

    Serial.print(" | L effort: ");
    Serial.print(left_effort, 2);

    Serial.print(" | R effort: ");
    Serial.print(right_effort, 2);


    // ---------------------------------------------------------
    // ENCODER TOTALS
    // ---------------------------------------------------------

    Serial.print(" | L ticks: ");
    Serial.print(enc.left_ticks_total);

    Serial.print(" | R ticks: ");
    Serial.print(enc.right_ticks_total);


    // ---------------------------------------------------------
    // ENCODER DELTAS
    // ---------------------------------------------------------

    Serial.print(" | L delta: ");
    Serial.print(enc.left_delta_ticks);

    Serial.print(" | R delta: ");
    Serial.print(enc.right_delta_ticks);


    // ---------------------------------------------------------
    // WHEEL SPEEDS
    // ---------------------------------------------------------

    Serial.print(" | L speed: ");
    Serial.print(enc.left_speed_mm_s, 1);

    Serial.print(" mm/s");

    Serial.print(" | R speed: ");
    Serial.print(enc.right_speed_mm_s, 1);

    Serial.print(" mm/s");


    // ---------------------------------------------------------
    // SPEED DIFFERENCE
    // ---------------------------------------------------------

    float speed_difference =
        fabsf(enc.left_speed_mm_s)
        -
        fabsf(enc.right_speed_mm_s);

    Serial.print(" | Diff: ");
    Serial.print(speed_difference, 1);

    Serial.println(" mm/s");
}


// =============================================================
// RUN MOTOR TEST
// =============================================================

void runMotorTest(
    const char* name,
    float left_effort,
    float right_effort
) {

    Serial.println();
    Serial.println("=============================================");
    Serial.print("[TEST] ");
    Serial.println(name);
    Serial.println("=============================================");


    // ---------------------------------------------------------
    // RESET ENCODERS
    // ---------------------------------------------------------

    encoders.reset();


    // ---------------------------------------------------------
    // START MOTORS
    // ---------------------------------------------------------

    motors.setRawEffort(
        left_effort,
        right_effort
    );


    // ---------------------------------------------------------
    // READ ENCODERS WHILE MOTORS RUN
    // ---------------------------------------------------------

    unsigned long last_sample_ms =
        millis();

    unsigned long test_start_ms =
        millis();


    for (int i = 0; i < NUM_SAMPLES; i++) {

        // Wait until 100 ms has elapsed.
        while (
            millis() - last_sample_ms
            <
            SAMPLE_INTERVAL_MS
        ) {
            delay(1);
        }


        unsigned long now_ms =
            millis();

        float dt =
            (
                now_ms
                -
                last_sample_ms
            )
            /
            1000.0f;

        last_sample_ms =
            now_ms;


        // -----------------------------------------------------
        // UPDATE THE REAL ENCODER CLASS
        // -----------------------------------------------------

        encoders.update(
            dt
        );


        // -----------------------------------------------------
        // KEEP MOTOR COMMAND ACTIVE
        // -----------------------------------------------------
        //
        // motors.cpp sends a periodic command every 100 ms.
        // Calling this here also exercises that production logic.
        // -----------------------------------------------------

        motors.setRawEffort(
            left_effort,
            right_effort
        );


        // -----------------------------------------------------
        // PRINT
        // -----------------------------------------------------

        printEncoderState(
            now_ms - test_start_ms,
            left_effort,
            right_effort
        );
    }


    // ---------------------------------------------------------
    // STOP
    // ---------------------------------------------------------

    motors.coast();


    // Capture any final encoder movement.
    delay(100);

    encoders.update(
        0.1f
    );


    // ---------------------------------------------------------
    // FINAL RESULTS
    // ---------------------------------------------------------

    EncoderState finalState =
        encoders.getState();


    Serial.println();
    Serial.println("============== FINAL ===============");

    Serial.print("Left total ticks:  ");
    Serial.println(
        finalState.left_ticks_total
    );

    Serial.print("Right total ticks: ");
    Serial.println(
        finalState.right_ticks_total
    );

    Serial.print("Left distance:      ");
    Serial.print(
        finalState.left_dist_mm,
        2
    );
    Serial.println(" mm");

    Serial.print("Right distance:     ");
    Serial.print(
        finalState.right_dist_mm,
        2
    );
    Serial.println(" mm");


    // ---------------------------------------------------------
    // COMPARE ABSOLUTE TICK COUNTS
    // ---------------------------------------------------------

    long left_abs =
        labs(
            (long)finalState.left_ticks_total
        );

    long right_abs =
        labs(
            (long)finalState.right_ticks_total
        );

    Serial.print("Absolute tick difference: ");

    Serial.println(
        left_abs - right_abs
    );

    Serial.println("====================================");
}


// =============================================================
// SETUP
// =============================================================

void setup() {

    Serial.begin(
        115200
    );

    delay(
        3000
    );


    Serial.println();
    Serial.println("=============================================");
    Serial.println(" MOTOR + ENCODER HARDWARE CLASS TEST");
    Serial.println("=============================================");


    // =========================================================
    // ENABLE SHARED ROBOT POWER
    // =========================================================

    pinMode(
        13,
        OUTPUT
    );

    digitalWrite(
        13,
        HIGH
    );

    delay(
        300
    );


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

    Serial.println(
        "[TEST] I2C started."
    );


    // =========================================================
    // MOTORS
    // =========================================================

    Serial.println(
        "[TEST] Starting Motors class..."
    );

    motors.begin();

    Serial.println(
        "[TEST] Motors initialized."
    );


    // =========================================================
    // ENCODERS
    // =========================================================

    Serial.println(
        "[TEST] Starting Encoders class..."
    );

    encoders.begin();

    Serial.println(
        "[TEST] Encoders initialized."
    );


    // =========================================================
    // SHOW CURRENT CONFIGURATION
    // =========================================================

    bool motor_left_inverted;
    bool motor_right_inverted;

    motors.getInverted(
        motor_left_inverted,
        motor_right_inverted
    );


    bool encoder_left_inverted;
    bool encoder_right_inverted;

    encoders.getInverted(
        encoder_left_inverted,
        encoder_right_inverted
    );


    Serial.println();

    Serial.printf(
        "Motor inversion:   L=%s R=%s\n",
        motor_left_inverted ? "true" : "false",
        motor_right_inverted ? "true" : "false"
    );

    Serial.printf(
        "Encoder inversion: L=%s R=%s\n",
        encoder_left_inverted ? "true" : "false",
        encoder_right_inverted ? "true" : "false"
    );


    Serial.println();
    Serial.println(
        "Starting test in 2 seconds..."
    );

    delay(
        2000
    );
}


// =============================================================
// LOOP
// =============================================================

void loop() {

    // =========================================================
    // FORWARD
    // =========================================================

    runMotorTest(
        "FORWARD",
        TEST_EFFORT,
        TEST_EFFORT
    );


    // =========================================================
    // STOP
    // =========================================================

    Serial.println();
    Serial.println(
        "[TEST] STOP"
    );

    motors.coast();

    delay(
        1500
    );


    // =========================================================
    // REVERSE
    // =========================================================

    runMotorTest(
        "REVERSE",
        -TEST_EFFORT,
        -TEST_EFFORT
    );


    // =========================================================
    // STOP
    // =========================================================

    Serial.println();
    Serial.println(
        "[TEST] STOP"
    );

    motors.coast();

    delay(
        2000
    );
}