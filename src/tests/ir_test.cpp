#include <Arduino.h>

// =============================================================
// IR SENSOR TEST
// ONE EMITTER + ALL 6 RECEIVERS
// =============================================================

const int POWER_ENABLE = 13;

// Emitter GPIO mappings: CH1 - CH6
const int EMITTER_PINS[6] = {
    15, 16, 17, 14, 18, 19
};

// Receiver ADC mappings: CH1 - CH6
const int RECEIVER_PINS[6] = {
    3, 8, 1, 10, 2, 9
};

// =============================================================
// CHOOSE WHICH EMITTER TO TEST
// 0 = CH1 = Left
// 1 = CH2 = Left 45
// 2 = CH3 = Front Left
// 3 = CH4 = Front Right
// 4 = CH5 = Right 45
// 5 = CH6 = Right
// =============================================================

// List one or more emitters, e.g. {2} or {0, 2, 4}.
// All listed emitters are switched on together.
const int EMITTER_TO_TEST[] = {0, 1,};

const int NUM_EMITTERS_TO_TEST =
    sizeof(EMITTER_TO_TEST) / sizeof(EMITTER_TO_TEST[0]);

// Number of ADC readings averaged
const int NUM_SAMPLES = 8;


// =============================================================
// READ AVERAGE ADC VALUE
// =============================================================

// =============================================================
// SWITCH ALL SELECTED EMITTERS ON OR OFF
// =============================================================

void setTestEmitters(int level) {

    for (int i = 0; i < NUM_EMITTERS_TO_TEST; i++) {
        digitalWrite(EMITTER_PINS[EMITTER_TO_TEST[i]], level);
    }
}


// =============================================================
// PRINT SELECTED EMITTERS, e.g. "CH1+CH3+CH5"
// =============================================================

void printTestEmitters() {

    for (int i = 0; i < NUM_EMITTERS_TO_TEST; i++) {

        if (i > 0) {
            Serial.print("+");
        }

        Serial.printf("CH%d", EMITTER_TO_TEST[i] + 1);
    }
}


int readAdcAvg(int pin) {

    long sum = 0;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        sum += analogRead(pin);
        delayMicroseconds(50);
    }

    return sum / NUM_SAMPLES;
}


// =============================================================
// SETUP
// =============================================================

void setup() {

    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("=============================================");
    Serial.println(" ONE IR EMITTER + ALL RECEIVERS TEST");
    Serial.println("=============================================");

    // Enable board power
    pinMode(POWER_ENABLE, OUTPUT);
    digitalWrite(POWER_ENABLE, HIGH);

    delay(200);


    // ---------------------------------------------------------
    // Configure all emitters
    // ---------------------------------------------------------

    for (int i = 0; i < 6; i++) {

        pinMode(EMITTER_PINS[i], OUTPUT);

        // Keep every emitter OFF initially
        digitalWrite(EMITTER_PINS[i], LOW);
    }


    // ---------------------------------------------------------
    // Configure all receivers
    // ---------------------------------------------------------

    for (int i = 0; i < 6; i++) {
        pinMode(RECEIVER_PINS[i], INPUT);
    }


    // ESP32-S3 ADC configuration
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);


    Serial.print("Testing emitter ");
    printTestEmitters();
    Serial.println();

    Serial.println("All 6 receivers active.");
    Serial.println();
}


// =============================================================
// LOOP
// =============================================================

void loop() {

    int ambient[6];
    int active[6];
    int deltas[6];


    // =========================================================
    // STEP 1: EMITTER OFF
    // =========================================================

    setTestEmitters(LOW);

    delayMicroseconds(300);


    // =========================================================
    // STEP 2: READ ALL 6 RECEIVERS WITH EMITTER OFF
    // =========================================================

    for (int i = 0; i < 6; i++) {
        ambient[i] = readAdcAvg(RECEIVER_PINS[i]);
    }


    // =========================================================
    // STEP 3: TURN ONLY ONE EMITTER ON
    // =========================================================

    setTestEmitters(HIGH);

    delayMicroseconds(300);


    // =========================================================
    // STEP 4: READ ALL 6 RECEIVERS WITH EMITTER ON
    // =========================================================

    for (int i = 0; i < 6; i++) {

        active[i] = readAdcAvg(RECEIVER_PINS[i]);

        deltas[i] = active[i] - ambient[i];
    }


    // =========================================================
    // STEP 5: TURN EMITTER BACK OFF
    // =========================================================

    setTestEmitters(LOW);


    // =========================================================
    // PRINT RESULTS
    // =========================================================

    Serial.print("Emitter ");
    printTestEmitters();

    Serial.printf(
        " | R0:%4d | R1:%4d | R2:%4d | R3:%4d | R4:%4d | R5:%4d\n",
        deltas[0],
        deltas[1],
        deltas[2],
        deltas[3],
        deltas[4],
        deltas[5]
    );

    delay(100);
}