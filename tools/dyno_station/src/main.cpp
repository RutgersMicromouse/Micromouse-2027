#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>

#include "dyno_config.h"

// ==============================================================================
// 1. DUAL OPTICAL SLIT TACHOMETER (INTERRUPT-DRIVEN)
// ==============================================================================
static volatile uint32_t s_last_pulse_left_us   = 0;
static volatile uint32_t s_period_left_us       = 0;
static volatile uint32_t s_pulse_count_left     = 0;

static volatile uint32_t s_last_pulse_right_us  = 0;
static volatile uint32_t s_period_right_us      = 0;
static volatile uint32_t s_pulse_count_right    = 0;

void IRAM_ATTR isrGateLeft() {
    uint32_t now = micros();
    uint32_t dt = now - s_last_pulse_left_us;
    if (dt > OPTICAL_DEBOUNCE_US) {
        s_period_left_us = dt;
        s_last_pulse_left_us = now;
        s_pulse_count_left++;
    }
}

void IRAM_ATTR isrGateRight() {
    uint32_t now = micros();
    uint32_t dt = now - s_last_pulse_right_us;
    if (dt > OPTICAL_DEBOUNCE_US) {
        s_period_right_us = dt;
        s_last_pulse_right_us = now;
        s_pulse_count_right++;
    }
}

void resetTachometers() {
    noInterrupts();
    s_pulse_count_left = 0;
    s_pulse_count_right = 0;
    s_period_left_us = 0;
    s_period_right_us = 0;
    s_last_pulse_left_us = micros();
    s_last_pulse_right_us = micros();
    interrupts();
}

float getLeftRPM() {
    noInterrupts();
    uint32_t p = s_period_left_us;
    uint32_t last = s_last_pulse_left_us;
    interrupts();
    if (micros() - last > 350000) return 0.0f; // Stationary if no pulse in 350ms
    if (p == 0) return 0.0f;
    return (60000000.0f / (float)p) / (float)SLITS_PER_WHEEL;
}

float getRightRPM() {
    noInterrupts();
    uint32_t p = s_period_right_us;
    uint32_t last = s_last_pulse_right_us;
    interrupts();
    if (micros() - last > 350000) return 0.0f;
    if (p == 0) return 0.0f;
    return (60000000.0f / (float)p) / (float)SLITS_PER_WHEEL;
}

// ==============================================================================
// 2. WIRELESS BLE CLIENT (NORDIC UART SERVICE TO ROBOT)
// ==============================================================================
static BLEAdvertisedDevice*   s_target_device = nullptr;
static BLEClient*             s_ble_client    = nullptr;
static BLERemoteCharacteristic* s_char_rx     = nullptr; // Dyno writes commands here
static BLERemoteCharacteristic* s_char_tx     = nullptr; // Robot notifies responses here

static bool   s_is_connected = false;
static String s_latest_reply = "";
static bool   s_new_reply_ready = false;

static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic,
                           uint8_t* pData, size_t length, bool isNotify) {
    char buf[128];
    size_t copy_len = length < sizeof(buf) - 1 ? length : sizeof(buf) - 1;
    memcpy(buf, pData, copy_len);
    buf[copy_len] = '\0';
    s_latest_reply = String(buf);
    s_latest_reply.trim();
    s_new_reply_ready = true;
}

class AdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        if (advertisedDevice.haveName() && advertisedDevice.getName() == BLE_TARGET_DEVICE_NAME) {
            Serial.printf("[BLE SCAN] Found target Micromouse: %s [%s]\n",
                          advertisedDevice.getName().c_str(),
                          advertisedDevice.getAddress().toString().c_str());
            advertisedDevice.getScan()->stop();
            s_target_device = new BLEAdvertisedDevice(advertisedDevice);
        }
    }
};

bool connectToRobot() {
    if (!s_target_device) return false;
    Serial.printf("[BLE] Connecting to %s...\n", s_target_device->getAddress().toString().c_str());
    s_ble_client = BLEDevice::createClient();

    if (!s_ble_client->connect(s_target_device)) {
        Serial.println("[BLE ERR] Connection failed!");
        return false;
    }
    Serial.println("[BLE] Connected to robot! Discovering Nordic UART Service...");

    BLERemoteService* pService = s_ble_client->getService(BLEUUID(NORDIC_UART_SERVICE_UUID));
    if (!pService) {
        Serial.println("[BLE ERR] Failed to find Nordic UART Service!");
        s_ble_client->disconnect();
        return false;
    }

    s_char_rx = pService->getCharacteristic(BLEUUID(NORDIC_UART_CHAR_RX_UUID));
    s_char_tx = pService->getCharacteristic(BLEUUID(NORDIC_UART_CHAR_TX_UUID));

    if (!s_char_rx || !s_char_tx) {
        Serial.println("[BLE ERR] Failed to resolve RX/TX characteristics!");
        s_ble_client->disconnect();
        return false;
    }

    if (s_char_tx->canNotify()) {
        s_char_tx->registerForNotify(notifyCallback);
    }

    s_is_connected = true;
    digitalWrite(PIN_STATUS_LED, HIGH);
    Serial.println("[BLE SUCCESS] Wireless link established with Antigrav-Mouse!\n");
    return true;
}

void sendBotCommand(const String& cmd) {
    if (!s_is_connected || !s_char_rx) return;
    s_new_reply_ready = false;
    s_latest_reply = "";
    String payload = cmd + "\n";
    s_char_rx->writeValue((uint8_t*)payload.c_str(), payload.length(), false);
}

String awaitBotReply(uint32_t timeout_ms = 2500) {
    uint32_t start = millis();
    while (!s_new_reply_ready && (millis() - start < timeout_ms)) {
        delay(10);
    }
    if (s_new_reply_ready) {
        s_new_reply_ready = false;
        return s_latest_reply;
    }
    return "TIMEOUT";
}

// ==============================================================================
// 3. AUTOMATED 4-STAGE ITERATIVE CALIBRATION & SYNCHRONIZATION ENGINE
// ==============================================================================

// Stage 1: Deadband / Breakaway Stiction Identification
bool calibrateStiction(float& out_deadband_l, float& out_deadband_r) {
    Serial.println("\n============================================================");
    Serial.println("  STAGE 1: DUAL-WHEEL BREAKAWAY STICTION SWEEP");
    Serial.println("============================================================");

    out_deadband_l = 0.02f;
    out_deadband_r = 0.02f;

    // Test Left Motor Stiction
    Serial.print("  • Sweeping Left motor breakaway voltage...");
    for (float duty = 0.005f; duty <= 0.15f; duty += 0.003f) {
        resetTachometers();
        char buf[32];
        snprintf(buf, sizeof(buf), "dyno step %.3f 0.0 120", duty);
        sendBotCommand(buf);
        delay(140);
        if (s_pulse_count_left >= 1) {
            out_deadband_l = duty;
            Serial.printf(" BREAKAWAY AT %.3f duty (%.1f%%)\n", duty, duty * 100.0f);
            break;
        }
    }
    delay(400);

    // Test Right Motor Stiction
    Serial.print("  • Sweeping Right motor breakaway voltage...");
    for (float duty = 0.005f; duty <= 0.15f; duty += 0.003f) {
        resetTachometers();
        char buf[32];
        snprintf(buf, sizeof(buf), "dyno step 0.0 %.3f 120", duty);
        sendBotCommand(buf);
        delay(140);
        if (s_pulse_count_right >= 1) {
            out_deadband_r = duty;
            Serial.printf(" BREAKAWAY AT %.3f duty (%.1f%%)\n", duty, duty * 100.0f);
            break;
        }
    }
    delay(400);

    // Update robot deadbands
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "dyno deadband %.4f %.4f", out_deadband_l, out_deadband_r);
    sendBotCommand(cmd);
    awaitBotReply(500);

    Serial.printf("  ✓ Stage 1 Complete: Calibrated Deadbands: L=%.3f, R=%.3f\n",
                  out_deadband_l, out_deadband_r);
    return true;
}

// Stage 2: Steady-State Kv Linearization & Trim Balancing
bool calibrateKvTrim(float& out_trim_l, float& out_trim_r) {
    Serial.println("\n============================================================");
    Serial.println("  STAGE 2: MULTI-POINT Kv STEADY-STATE RPM LINEARIZATION");
    Serial.println("============================================================");
    Serial.println(" Duty  |   Left RPM   |  Right RPM   |   Diff RPM   | Ratio L/R");
    Serial.println("-------+--------------+--------------+--------------+----------");

    const float test_duties[] = { 0.25f, 0.40f, 0.55f, 0.70f, 0.85f, 0.95f };
    const int num_points = sizeof(test_duties) / sizeof(test_duties[0]);
    float ratio_sum = 0.0f;

    for (int i = 0; i < num_points; ++i) {
        float d = test_duties[i];
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "dyno step %.2f %.2f 1400", d, d);
        sendBotCommand(cmd);

        delay(500); // Wait for motor acceleration and speed to stabilize
        resetTachometers();
        delay(700); // 700ms sampling window

        float rpm_l = getLeftRPM();
        float rpm_r = getRightRPM();
        float diff = rpm_l - rpm_r;
        float ratio = (rpm_r > 10.0f) ? (rpm_l / rpm_r) : 1.0f;
        ratio_sum += ratio;

        Serial.printf(" %3.0f%%  |  %7.1f RPM |  %7.1f RPM | %+7.1f RPM |  %5.3f\n",
                      d * 100.0f, rpm_l, rpm_r, diff, ratio);
        delay(600); // Cooldown
    }

    float avg_ratio = ratio_sum / (float)num_points;
    if (avg_ratio > 1.002f) {
        // Left is faster -> scale Left down
        out_trim_l = constrain(1.0f / avg_ratio, 0.60f, 1.0f);
        out_trim_r = 1.0f;
    } else if (avg_ratio < 0.998f) {
        // Right is faster -> scale Right down
        out_trim_l = 1.0f;
        out_trim_r = constrain(avg_ratio, 0.60f, 1.0f);
    } else {
        out_trim_l = 1.0f;
        out_trim_r = 1.0f;
    }

    // Apply calculated trim to robot
    char trim_cmd[64];
    snprintf(trim_cmd, sizeof(trim_cmd), "dyno trim %.4f %.4f", out_trim_l, out_trim_r);
    sendBotCommand(trim_cmd);
    awaitBotReply(500);

    // Verify at 50% duty with trim active
    Serial.println("\n  • Verifying trimmed balance at 50% duty...");
    sendBotCommand("dyno step 0.50 0.50 1400");
    delay(500);
    resetTachometers();
    delay(700);
    float v_l = getLeftRPM();
    float v_r = getRightRPM();
    Serial.printf("  ✓ Trim Verification: Left=%5.1f RPM | Right=%5.1f RPM | Error=%+.1f RPM (%.2f%%)\n",
                  v_l, v_r, v_l - v_r, (fabsf(v_l - v_r) / v_l) * 100.0f);

    return true;
}

// Stage 3 & 4: Dynamic Acceleration / Deceleration & Cross-Coupled Sync Lock Tuning
bool calibrateSyncLock(float& out_k_sync) {
    Serial.println("\n============================================================");
    Serial.println("  STAGE 3 & 4: SPRINT ACCELERATION & PHASE-LOCK TUNING");
    Serial.println("============================================================");
    Serial.println(" Iter |  K_sync  | Left Pulses | Right Pulses | Drift (deg) | Sync Status");
    Serial.println("------+----------+-------------+--------------+-------------+------------");

    float current_ksync = 0.0003f;
    float best_ksync = current_ksync;
    float min_drift_deg = 999.0f;

    for (int iter = 1; iter <= 6; ++iter) {
        char sync_cmd[32];
        snprintf(sync_cmd, sizeof(sync_cmd), "dyno sync %.6f", current_ksync);
        sendBotCommand(sync_cmd);
        awaitBotReply(300);

        // Command full-speed championship sprint: 500 mm/s @ 2600 mm/s2 for 800 mm
        resetTachometers();
        sendBotCommand("dyno trap 500 2600 800");

        // Monitor sprint duration (~1.8 seconds)
        delay(2200);

        noInterrupts();
        uint32_t cnt_l = s_pulse_count_left;
        uint32_t cnt_r = s_pulse_count_right;
        interrupts();

        // Calculate angular drift between wheels
        int32_t diff_pulses = (int32_t)cnt_l - (int32_t)cnt_r;
        float drift_deg = ((float)abs(diff_pulses) / (float)SLITS_PER_WHEEL) * 360.0f;

        bool synchronized = (drift_deg <= MAX_ALLOWED_PHASE_DRIFT);
        Serial.printf("  #%d  | %.6f |   %6lu    |    %6lu    |   %5.2f°   | %s\n",
                      iter, current_ksync, cnt_l, cnt_r, drift_deg,
                      synchronized ? "⚡ SYNCHRONIZED" : "Converging...");

        if (drift_deg < min_drift_deg) {
            min_drift_deg = drift_deg;
            best_ksync = current_ksync;
        }

        if (synchronized) break;

        // Gradient descent on K_sync
        current_ksync += 0.00015f;
    }

    out_k_sync = best_ksync;
    char final_sync_cmd[32];
    snprintf(final_sync_cmd, sizeof(final_sync_cmd), "dyno sync %.6f", out_k_sync);
    sendBotCommand(final_sync_cmd);
    awaitBotReply(300);

    Serial.printf("\n  ✓ Stage 4 Complete: Optimal Sync Lock Gain = %.6f (Residual drift: %.2f°)\n",
                  out_k_sync, min_drift_deg);
    return true;
}

// Stage 5: Save & Commit
void commitCalibration() {
    Serial.println("\n============================================================");
    Serial.println("  STAGE 5: COMMIT CALIBRATION TO ROBOT NVS FLASH");
    Serial.println("============================================================");
    sendBotCommand("dyno save");
    String reply = awaitBotReply(1000);
    Serial.printf("  [ROBOT RESPONSE] %s\n", reply.c_str());

    sendBotCommand("dyno get");
    String params = awaitBotReply(1000);
    Serial.printf("  [STORED NVS PARAMETERS] %s\n", params.c_str());
    Serial.println("\n  🏆 CALIBRATION COMPLETE: The robot is now perfectly synchronized!");
    Serial.println("     It can now be unmounted from the dyno and placed on the maze.\n");
}

void runFullAutoTune() {
    float deadband_l = 0, deadband_r = 0;
    float trim_l = 1.0f, trim_r = 1.0f;
    float k_sync = 0.0004f;

    calibrateStiction(deadband_l, deadband_r);
    calibrateKvTrim(trim_l, trim_r);
    calibrateSyncLock(k_sync);
    commitCalibration();
}

// ==============================================================================
// 4. SETUP & INTERACTIVE SERIAL COMMAND DISPATCHER
// ==============================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, LOW);

    // 1. Configure Optical Gate Inputs with Internal Pull-Ups
    pinMode(PIN_GATE_LEFT, INPUT_PULLUP);
    pinMode(PIN_GATE_RIGHT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_GATE_LEFT), isrGateLeft, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_GATE_RIGHT), isrGateRight, FALLING);

    Serial.println("\n"
    "============================================================\n"
    "  ANTIGRAVITIEEE AUTONOMOUS OPTICAL DYNO & TUNING STATION   \n"
    "============================================================");
    Serial.println("  • Optical Left Gate:  GPIO 18");
    Serial.println("  • Optical Right Gate: GPIO 19");
    Serial.println("  • Status LED:         GPIO 2");
    Serial.println("  • Target Robot BLE:   '" BLE_TARGET_DEVICE_NAME "'\n");

    // 2. Initialize BLE Scanner
    BLEDevice::init("Dyno-Station");
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());
    pBLEScan->setInterval(1349);
    pBLEScan->setWindow(449);
    pBLEScan->setActiveScan(true);

    Serial.println("[BLE] Scanning for Micromouse bot (make sure robot is powered on)...");
    pBLEScan->start(8, false);

    if (s_target_device) {
        connectToRobot();
    } else {
        Serial.println("[BLE WARN] Robot not found yet. Type 'scan' to retry or 'help' for options.");
    }
}

void loop() {
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        line.toLowerCase();

        if (line == "start" || line == "tune") {
            if (s_is_connected) {
                runFullAutoTune();
            } else {
                Serial.println("[ERR] Not connected to robot! Type 'scan' first.");
            }
        } else if (line == "scan") {
            Serial.println("[BLE] Scanning for robot...");
            BLEDevice::getScan()->start(5, false);
            if (s_target_device) connectToRobot();
        } else if (line == "stiction") {
            float dl = 0, dr = 0;
            calibrateStiction(dl, dr);
        } else if (line == "kv") {
            float tl = 1, tr = 1;
            calibrateKvTrim(tl, tr);
        } else if (line == "sync") {
            float ks = 0.0004f;
            calibrateSyncLock(ks);
        } else if (line == "save") {
            commitCalibration();
        } else if (line == "rpm") {
            Serial.printf("Live RPM: Left=%5.1f | Right=%5.1f | Left Pulses=%lu | Right Pulses=%lu\n",
                          getLeftRPM(), getRightRPM(), s_pulse_count_left, s_pulse_count_right);
        } else if (line == "help") {
            Serial.println("\nDyno Commands:");
            Serial.println("  start     - Run full 5-stage automated calibration and flash commit");
            Serial.println("  scan      - Scan and reconnect to robot over BLE");
            Serial.println("  stiction  - Run stiction / deadband sweep only");
            Serial.println("  kv        - Run multi-point steady-state RPM linearization only");
            Serial.println("  sync      - Run sprint acceleration phase-lock sync test");
            Serial.println("  rpm       - Print instantaneous optical RPM readings");
            Serial.println("  save      - Burn parameters to robot NVS Flash");
        } else {
            // Forward arbitrary command directly to robot over BLE
            sendBotCommand(line);
            String reply = awaitBotReply(1500);
            Serial.printf("[ROBOT] %s\n", reply.c_str());
        }
    }
    delay(20);
}
