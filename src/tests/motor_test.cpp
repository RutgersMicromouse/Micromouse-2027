#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>
#include "driver/pcnt.h"

// =============================================================
// MOTOR SETTINGS
// =============================================================

const int POWER_ENABLE   = 13;
const int SDA_PIN        = 21;
const int SCL_PIN        = 20;

const int RIGHT_MOTOR_CH = 1;
const int LEFT_MOTOR_CH  = 2;

MotoronI2C mc;


// =============================================================
// BALANCED MOTOR SPEEDS
// =============================================================

// Right motor is our reference motor.
//
// From previous testing:
// Forward:
//   Right = +200
//   Left  = +205
//
// Reverse:
//   Right = -200
//   Left  = -204

const int RIGHT_FORWARD_SPEED = 200;
const int LEFT_FORWARD_SPEED  = 205;

const int RIGHT_REVERSE_SPEED = -200;
const int LEFT_REVERSE_SPEED  = -204;


// =============================================================
// ENCODER SETTINGS
// =============================================================

// Right motor encoder
const int PIN_ENC_R_A = 4;
const int PIN_ENC_R_B = 5;

// Left motor encoder
const int PIN_ENC_L_A = 6;
const int PIN_ENC_L_B = 7;


// =============================================================
// TEST SETTINGS
// =============================================================

const int SAMPLE_INTERVAL_MS = 100;
const int TEST_DURATION_MS   = 3000;

const int NUM_SAMPLES =
    TEST_DURATION_MS / SAMPLE_INTERVAL_MS;


// =============================================================
// MOTORON ERROR CHECK
// =============================================================

void checkMotoronErrors() {

    uint16_t statusFlags = mc.getStatusFlags();

    if (statusFlags) {
        Serial.print("[WARNING] Motoron Status Flags: 0x");
        Serial.println(statusFlags, HEX);
    }
}


// =============================================================
// ENCODER INITIALIZATION
// =============================================================

void initEncoder(pcnt_unit_t unit, int pinA, int pinB) {

    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);


    // ---------------------------------------------------------
    // Encoder Channel A
    // ---------------------------------------------------------

    pcnt_config_t configA = {};

    configA.pulse_gpio_num = pinA;
    configA.ctrl_gpio_num  = pinB;

    configA.lctrl_mode = PCNT_MODE_KEEP;
    configA.hctrl_mode = PCNT_MODE_REVERSE;

    configA.pos_mode = PCNT_COUNT_INC;
    configA.neg_mode = PCNT_COUNT_DEC;

    configA.counter_h_lim = 32767;
    configA.counter_l_lim = -32768;

    configA.unit    = unit;
    configA.channel = PCNT_CHANNEL_0;

    pcnt_unit_config(&configA);


    // ---------------------------------------------------------
    // Encoder Channel B
    // ---------------------------------------------------------

    pcnt_config_t configB = {};

    configB.pulse_gpio_num = pinB;
    configB.ctrl_gpio_num  = pinA;

    configB.lctrl_mode = PCNT_MODE_REVERSE;
    configB.hctrl_mode = PCNT_MODE_KEEP;

    configB.pos_mode = PCNT_COUNT_INC;
    configB.neg_mode = PCNT_COUNT_DEC;

    configB.counter_h_lim = 32767;
    configB.counter_l_lim = -32768;

    configB.unit    = unit;
    configB.channel = PCNT_CHANNEL_1;

    pcnt_unit_config(&configB);


    // Filter very short/noisy pulses
    pcnt_set_filter_value(unit, 100);
    pcnt_filter_enable(unit);


    // Reset and start encoder
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);
}


// =============================================================
// READ ENCODERS FOR 3 SECONDS
// =============================================================

void readEncodersFor3Seconds(
    int leftCommand,
    int rightCommand
) {

    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_1);


    int16_t previousLeftTicks  = 0;
    int16_t previousRightTicks = 0;


    Serial.println();
    Serial.println(
        "Time | Left Cmd | Right Cmd | Left Ticks | Right Ticks | Left ticks/ms | Right ticks/ms | Difference"
    );

    Serial.println(
        "---------------------------------------------------------------------------------------------------"
    );


    for (int i = 0; i < NUM_SAMPLES; i++) {

        delay(SAMPLE_INTERVAL_MS);


        // -----------------------------------------------------
        // Read cumulative encoder counts
        // -----------------------------------------------------

        int16_t leftTicks  = 0;
        int16_t rightTicks = 0;

        pcnt_get_counter_value(
            PCNT_UNIT_0,
            &leftTicks
        );

        pcnt_get_counter_value(
            PCNT_UNIT_1,
            &rightTicks
        );


        // -----------------------------------------------------
        // Calculate ticks during THIS 100 ms interval
        // -----------------------------------------------------

        int leftDelta =
            leftTicks - previousLeftTicks;

        int rightDelta =
            rightTicks - previousRightTicks;


        previousLeftTicks  = leftTicks;
        previousRightTicks = rightTicks;


        // -----------------------------------------------------
        // Convert to ticks per millisecond
        // -----------------------------------------------------

        float leftTicksPerMs =
            abs(leftDelta) /
            (float)SAMPLE_INTERVAL_MS;

        float rightTicksPerMs =
            abs(rightDelta) /
            (float)SAMPLE_INTERVAL_MS;


        // Positive difference:
        // left wheel is faster
        //
        // Negative difference:
        // right wheel is faster

        float difference =
            leftTicksPerMs - rightTicksPerMs;


        // -----------------------------------------------------
        // Print results
        // -----------------------------------------------------

        Serial.print((i + 1) * SAMPLE_INTERVAL_MS);
        Serial.print(" ms");

        Serial.print(" | Lcmd: ");
        Serial.print(leftCommand);

        Serial.print(" | Rcmd: ");
        Serial.print(rightCommand);

        Serial.print(" | L: ");
        Serial.print(leftTicks);

        Serial.print(" | R: ");
        Serial.print(rightTicks);

        Serial.print(" | L rate: ");
        Serial.print(leftTicksPerMs, 3);

        Serial.print(" ticks/ms");

        Serial.print(" | R rate: ");
        Serial.print(rightTicksPerMs, 3);

        Serial.print(" ticks/ms");

        Serial.print(" | Diff: ");
        Serial.print(difference, 3);

        Serial.println(" ticks/ms");
    }


    // =========================================================
    // FINAL AVERAGE
    // =========================================================

    int16_t finalLeftTicks  = 0;
    int16_t finalRightTicks = 0;

    pcnt_get_counter_value(
        PCNT_UNIT_0,
        &finalLeftTicks
    );

    pcnt_get_counter_value(
        PCNT_UNIT_1,
        &finalRightTicks
    );


    float averageLeftRate =
        abs(finalLeftTicks) /
        (float)TEST_DURATION_MS;

    float averageRightRate =
        abs(finalRightTicks) /
        (float)TEST_DURATION_MS;


    Serial.println();
    Serial.println("============== AVERAGE ==============");

    Serial.print("Left:  ");
    Serial.print(averageLeftRate, 3);
    Serial.println(" ticks/ms");

    Serial.print("Right: ");
    Serial.print(averageRightRate, 3);
    Serial.println(" ticks/ms");

    Serial.print("Difference: ");
    Serial.print(
        averageLeftRate - averageRightRate,
        3
    );
    Serial.println(" ticks/ms");

    Serial.println("=====================================");
    Serial.println();
}


// =============================================================
// SETUP
// =============================================================

void setup() {

    Serial.begin(115200);
    delay(3000);

    Serial.println();
    Serial.println("=============================================");
    Serial.println(" MOTOR + ENCODER BALANCE TEST");
    Serial.println("=============================================");


    // ---------------------------------------------------------
    // Enable motor power
    // ---------------------------------------------------------

    pinMode(POWER_ENABLE, OUTPUT);
    digitalWrite(POWER_ENABLE, HIGH);

    delay(300);


    // ---------------------------------------------------------
    // Initialize Motoron
    // ---------------------------------------------------------

    Wire.begin(SDA_PIN, SCL_PIN);

    mc.reinitialize();
    mc.clearResetFlag();

    mc.disableCommandTimeout();

    mc.setErrorResponse(
        MOTORON_ERROR_RESPONSE_COAST
    );


    // ---------------------------------------------------------
    // Motor acceleration/deceleration
    // ---------------------------------------------------------

    mc.setMaxAcceleration(
        RIGHT_MOTOR_CH,
        100
    );

    mc.setMaxDeceleration(
        RIGHT_MOTOR_CH,
        100
    );

    mc.setMaxAcceleration(
        LEFT_MOTOR_CH,
        100
    );

    mc.setMaxDeceleration(
        LEFT_MOTOR_CH,
        100
    );


    // ---------------------------------------------------------
    // Initialize encoders
    // ---------------------------------------------------------

    // PCNT Unit 0 = Left
    initEncoder(
        PCNT_UNIT_0,
        PIN_ENC_L_A,
        PIN_ENC_L_B
    );


    // PCNT Unit 1 = Right
    initEncoder(
        PCNT_UNIT_1,
        PIN_ENC_R_A,
        PIN_ENC_R_B
    );


    Serial.println("Motoron initialized.");
    Serial.println("Encoders initialized.");

    Serial.println();

    Serial.println("Current calibration:");

    Serial.print("Forward: L=");
    Serial.print(LEFT_FORWARD_SPEED);

    Serial.print(" R=");
    Serial.println(RIGHT_FORWARD_SPEED);


    Serial.print("Reverse: L=");
    Serial.print(LEFT_REVERSE_SPEED);

    Serial.print(" R=");
    Serial.println(RIGHT_REVERSE_SPEED);

    Serial.println();
}


// =============================================================
// LOOP
// =============================================================

void loop() {

    mc.clearResetFlag();


    // =========================================================
    // FORWARD TEST
    // =========================================================

    Serial.println();
    Serial.println("=============================================");
    Serial.println("[TEST] FORWARD");
    Serial.println("=============================================");

    mc.setSpeed(
        RIGHT_MOTOR_CH,
        RIGHT_FORWARD_SPEED
    );

    mc.setSpeed(
        LEFT_MOTOR_CH,
        LEFT_FORWARD_SPEED
    );


    checkMotoronErrors();


    readEncodersFor3Seconds(
        LEFT_FORWARD_SPEED,
        RIGHT_FORWARD_SPEED
    );


    // =========================================================
    // STOP
    // =========================================================

    Serial.println("[TEST] STOP");

    mc.setSpeed(RIGHT_MOTOR_CH, 0);
    mc.setSpeed(LEFT_MOTOR_CH, 0);

    delay(1500);


    // =========================================================
    // REVERSE TEST
    // =========================================================

    Serial.println();
    Serial.println("=============================================");
    Serial.println("[TEST] REVERSE");
    Serial.println("=============================================");

    mc.setSpeed(
        RIGHT_MOTOR_CH,
        RIGHT_REVERSE_SPEED
    );

    mc.setSpeed(
        LEFT_MOTOR_CH,
        LEFT_REVERSE_SPEED
    );


    checkMotoronErrors();


    readEncodersFor3Seconds(
        LEFT_REVERSE_SPEED,
        RIGHT_REVERSE_SPEED
    );


    // =========================================================
    // STOP
    // =========================================================

    Serial.println("[TEST] STOP");

    mc.setSpeed(RIGHT_MOTOR_CH, 0);
    mc.setSpeed(LEFT_MOTOR_CH, 0);

    delay(2000);
}