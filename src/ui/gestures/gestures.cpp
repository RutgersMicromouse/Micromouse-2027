#include <functional>
#include "robot.h"
#include "ui/status_led/status_led.h"
#include "ui/actions/actions.h"
#include "ui/gestures/gestures.h"
#include "ui/console/console.h"
#include "wireless/ble_debug/ble_debug.h"
#include "wireless/wifi_ota/wifi_ota.h"

// ==============================================================================
// HAND-WAVE DETECTOR
// ==============================================================================

// Samples in a row needed before believing the hand arrived / left (2 samples = ~40 ms)
static const uint8_t DEBOUNCE_SAMPLES = 2;

GestureInput::GestureInput() {
    reset(0);
}

void GestureInput::reset(uint32_t now_ms) {
    resting_level_ = -1.0f; // Not learned yet
    hand_present_ = false;
    confirm_samples_ = 0;
    wave_count_ = 0;
    wave_counted_flag_ = false;
    started_ms_ = now_ms;
    hand_since_ms_ = now_ms;
    last_wave_end_ms_ = now_ms;
}

uint8_t GestureInput::update(uint16_t front_reading, uint32_t now_ms) {
    float reading = (float)front_reading;

    // Warm-up: just learn the resting level, so nothing that happens while the robot is being
    // put down can count as a wave
    if (resting_level_ < 0.0f) resting_level_ = reading;
    if (now_ms - started_ms_ < GESTURE_WARMUP_MS) {
        resting_level_ += (reading - resting_level_) * 0.2f;
        return 0;
    }

    // How far above the resting level counts as "a hand is there" (with hysteresis on the way out)
    float rise = resting_level_ * GESTURE_RISE_RATIO;
    if (rise < (float)GESTURE_MIN_RISE) rise = (float)GESTURE_MIN_RISE;
    float arrive_level = resting_level_ + rise;
    float leave_level  = resting_level_ + rise * 0.5f;

    if (!hand_present_) {
        if (reading > arrive_level) {
            if (++confirm_samples_ >= DEBOUNCE_SAMPLES) {
                hand_present_ = true;
                hand_since_ms_ = now_ms;
                confirm_samples_ = 0;
            }
        } else {
            confirm_samples_ = 0;
            resting_level_ += (reading - resting_level_) * 0.05f; // Slowly follow ambient changes
        }

        // The operator has stopped waving: report the total
        if (wave_count_ > 0 && !hand_present_ && (now_ms - last_wave_end_ms_ > GESTURE_SEQUENCE_GAP_MS)) {
            uint8_t total = wave_count_;
            wave_count_ = 0;
            return total;
        }
        return 0;
    }

    // A hand is currently in front of the sensors
    if (reading < leave_level) {
        if (++confirm_samples_ >= DEBOUNCE_SAMPLES) {
            hand_present_ = false;
            confirm_samples_ = 0;
            last_wave_end_ms_ = now_ms;

            if (now_ms - hand_since_ms_ <= GESTURE_MAX_WAVE_MS) {
                wave_count_++;
                wave_counted_flag_ = true;
            } else {
                wave_count_ = 0; // Holding a hand there is the "never mind" gesture
            }
        }
    } else {
        confirm_samples_ = 0;
        if (now_ms - hand_since_ms_ > GESTURE_REBASE_MS) {
            // Not a hand: the robot was moved and is now looking at something closer
            resting_level_ = reading;
            hand_present_ = false;
            wave_count_ = 0;
            last_wave_end_ms_ = now_ms;
        }
    }
    return 0;
}

float GestureInput::getHandLevel() const {
    float rise = resting_level_ * GESTURE_RISE_RATIO;
    if (rise < (float)GESTURE_MIN_RISE) rise = (float)GESTURE_MIN_RISE;
    return resting_level_ + rise;
}

bool GestureInput::consumeWaveCounted() {
    bool counted = wave_counted_flag_;
    wave_counted_flag_ = false;
    return counted;
}

// ==============================================================================
// HAND-WAVE CONTROLS
// ==============================================================================

namespace GestureUI {

static const uint8_t WAVES_CALIBRATE_IR = 5;
static const uint8_t WAVES_CLEAR_MAZE   = 6;

static GestureInput s_input;

void restart() {
    s_input.reset(millis());
}

// Rapid-blink countdown. Returns false if the operator covered the sensors to cancel.
static bool countdown(StatusLED::Color color, uint32_t duration_ms) {
    uint32_t start_ms = millis();
    while (millis() - start_ms < duration_ms) {
        bool led_on = ((millis() - start_ms) / 100) % 2 == 0;
        StatusLED::set(led_on ? color : StatusLED::OFF);

        RobotTelemetry telemetry;
        if (getTelemetry(telemetry)) {
            s_input.update(telemetry.ir.front_center, millis());
        }
        if (s_input.isHandPresent()) {
            Serial.println("[UI] Cancelled by hand.");
            StatusLED::flash(StatusLED::RED, 2, 100);
            return false;
        }
        delay(20);
    }
    return true;
}

static void perform(uint8_t waves) {
    Serial.printf("[UI] Counted %d hand wave(s).\n", waves);

    if (waves >= 1 && waves <= Actions::MODE_COUNT) {
        Actions::RunMode mode = (Actions::RunMode)(waves - 1);
        Actions::selectMode(mode);
        StatusLED::flash(Actions::modeColor(mode), waves, 150);
        if (mode != Actions::MODE_SEARCH) {
            // Speed tier the run will use: 1, 2, or 3 blue blinks
            delay(300);
            StatusLED::flash(StatusLED::BLUE, Actions::getSpeedTier(), 150);
        }
        if (countdown(Actions::modeColor(mode), GESTURE_LAUNCH_DELAY_MS)) {
            Actions::launchSelectedRun();
        }
    } else if (waves == WAVES_CALIBRATE_IR) {
        StatusLED::flash(StatusLED::YELLOW, waves, 150);
        if (countdown(StatusLED::YELLOW, GESTURE_LAUNCH_DELAY_MS)) {
            Actions::calibrateIR();
        }
    } else if (waves == WAVES_CLEAR_MAZE) {
        Actions::clearSavedMaze();
    } else {
        StatusLED::flash(StatusLED::RED, 2, 100); // No action for that many waves
    }

    Actions::showSelectedMode();
    restart();
}

void describe(char* buf, size_t size, const IRReadings& ir) {
    snprintf(buf, size, "WAVES: front sensors read %d now (FL=%d FR=%d), %.0f at rest, a hand must exceed %.0f. Hand %s, %d wave(s) counted so far",
             (int)ir.front_center, (int)ir.front_left, (int)ir.front_right,
             s_input.getRestingLevel(), s_input.getHandLevel(),
             s_input.isHandPresent() ? "SEEN" : "not seen", (int)s_input.getPendingCount());
}

void update(const IRReadings& ir) {
    static bool hand_was_present = false;
    uint8_t waves = s_input.update(ir.front_center, millis());

    // Say on the serial monitor what is being seen, so waves can be checked without the LED
    if (s_input.isHandPresent() != hand_was_present) {
        hand_was_present = s_input.isHandPresent();
        if (hand_was_present) Serial.printf("[UI] Hand seen (front sensors read %d)\n", (int)ir.front_center);
    }

    if (s_input.consumeWaveCounted()) {
        Serial.printf("[UI] Wave %d counted\n", (int)s_input.getPendingCount());
        // Blink white so the operator can see each wave being counted
        StatusLED::set(StatusLED::WHITE);
        delay(60);
        Actions::showSelectedMode();
    }

    if (waves > 0) {
        perform(waves);
    }
}

} // namespace GestureUI
