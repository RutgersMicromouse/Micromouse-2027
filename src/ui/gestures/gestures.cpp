#include <functional>
#include "robot.h"
#include "ui/status_led/status_led.h"
#include "ui/actions/actions.h"
#include "ui/gestures/gestures.h"
#include "ui/console/console.h"
#include "wireless/ble_debug/ble_debug.h"
#include "wireless/wifi_ota/wifi_ota.h"

// ==============================================================================
// HAND DETECTOR
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
    started_ms_ = now_ms;
    hand_since_ms_ = now_ms;
}

bool GestureInput::update(uint16_t sensor_reading, uint32_t now_ms) {
    float reading = (float)sensor_reading;

    // Warm-up: just learn the resting level, so nothing that happens while the robot is being
    // put down can count as a hand
    if (resting_level_ < 0.0f) resting_level_ = reading;
    if (isWarmingUp(now_ms)) {
        resting_level_ += (reading - resting_level_) * 0.2f;
        return false;
    }

    // A hand is a reading well away from the resting level, EITHER way. In the open a hand is the
    // only thing reflecting, so the reading rises. Beside a wall it is the other way round: the
    // hand hides the wall and reflects less than it (white walls especially: 530 down to 250 on
    // the robot, 2026-10-10), so the reading drops. Half the distance counts on the way out.
    const float rise = getHandLevel() - resting_level_;
    const float drop = resting_level_ - getDropLevel();
    const float away = reading - resting_level_;
    const bool hand_here = (away > rise) || (away < -drop);
    const bool hand_gone = (away < rise * 0.5f) && (away > -drop * 0.5f);

    if (!hand_present_) {
        if (hand_here) {
            if (++confirm_samples_ >= DEBOUNCE_SAMPLES) {
                hand_present_ = true;
                hand_since_ms_ = now_ms;
                confirm_samples_ = 0;
            }
        } else {
            confirm_samples_ = 0;
            resting_level_ += (reading - resting_level_) * 0.05f; // Slowly follow ambient changes
        }
        return false;
    }

    // A hand is currently in front of the sensors
    if (hand_gone) {
        if (++confirm_samples_ >= DEBOUNCE_SAMPLES) {
            hand_present_ = false;
            confirm_samples_ = 0;
            return true;
        }
    } else {
        confirm_samples_ = 0;
        if (now_ms - hand_since_ms_ > GESTURE_REBASE_MS) {
            // Not a hand: the robot was moved and is now looking at something else. This is
            // also the "never mind" gesture: keep the hand there and nothing happens.
            resting_level_ = reading;
            hand_present_ = false;
        }
    }
    return false;
}

bool GestureInput::isWarmingUp(uint32_t now_ms) const {
    return now_ms - started_ms_ < GESTURE_WARMUP_MS;
}

float GestureInput::getHandLevel() const {
    float rise = resting_level_ * GESTURE_RISE_RATIO;
    if (rise < (float)GESTURE_MIN_RISE) rise = (float)GESTURE_MIN_RISE;
    return resting_level_ + rise;
}

float GestureInput::getDropLevel() const {
    float drop = resting_level_ * GESTURE_DROP_RATIO;
    if (drop < (float)GESTURE_MIN_DROP) drop = (float)GESTURE_MIN_DROP;
    return resting_level_ - drop; // Below zero when there is no wall to hide: then only a rise counts
}

// ==============================================================================
// HAND CONTROLS
// ==============================================================================

namespace GestureUI {

// What can be chosen, in the order the LED goes through them
enum Stage : uint8_t { STAGE_SEARCH = 0, STAGE_SPEED_RUN, STAGE_CALIBRATE, STAGE_FORGET_MAZE, STAGE_COUNT };

static const char* const kStageName[STAGE_COUNT] = { "SEARCH", "SPEED RUN", "RESET SENSORS + CALIBRATE", "FORGET THE MAZE" };
static const StatusLED::Color kStageColor[STAGE_COUNT] = { StatusLED::GREEN, StatusLED::YELLOW, StatusLED::WHITE, StatusLED::BLUE };

static const uint8_t kSpeedPercents[]   = GESTURE_SPEED_PERCENTS;
static const uint8_t kSpeedBrightness[] = GESTURE_SPEED_BRIGHTNESS;
static const uint8_t SPEED_LEVELS = sizeof(kSpeedPercents) / sizeof(kSpeedPercents[0]);
static_assert(sizeof(kSpeedBrightness) == sizeof(kSpeedPercents), "one LED brightness per speed level");

static const StatusLED::Color READY_COLOR = StatusLED::CYAN;
static const uint8_t WAIT_BRIGHTNESS = 3;   // "Not listening yet": the ready colour, very dim

// Where the choosing stands
enum Mode : uint8_t {
    MODE_READY,     // Solid cyan: nothing is being chosen. A hand starts the stages.
    MODE_STAGE,     // Blinking a stage's colour: a hand = next stage, left alone = that stage
    MODE_SPEED      // Blinking yellow at a speed level's brightness: a hand = next level, left alone = go
};

static GestureInput s_input;                  // The front sensors
static Mode s_mode = MODE_READY;
static uint8_t s_stage = STAGE_SEARCH;
static uint8_t s_speed_level = 0;             // Place in kSpeedPercents; kept from one speed run to the next
static uint32_t s_blink_start_ms = 0;         // When the blinks of the present stage / level began
static float s_still_heading_deg = 0.0f;      // Heading when the robot was last seen to move

// What the LED is showing and what that means, kept for the phone app's hand-control display
static StatusLED::Color s_led_color = StatusLED::OFF;
static uint8_t s_led_brightness = 0;
static char s_led_meaning[80] = "Starting up";

static void show(StatusLED::Color color, uint8_t brightness, const char* meaning) {
    static uint32_t written_ms = 0;
    strlcpy(s_led_meaning, meaning, sizeof(s_led_meaning));
    // Other code blinks the LED too (run results, calibration), so write it again now and then
    const bool same = color.red == s_led_color.red && color.green == s_led_color.green && color.blue == s_led_color.blue &&
                      brightness == s_led_brightness;
    if (same && millis() - written_ms < 250) return;
    written_ms = millis();
    s_led_color = color;
    s_led_brightness = brightness;
    StatusLED::set(color, brightness);
}

// True if the robot has turned since this was last asked, or is tipped out of level: it is in
// somebody's hands
static bool robotWasMoved() {
    if (g_imu.getTiltDeg() > GESTURE_LIFTED_DEG) return true;
    const float heading = g_imu.getHeadingDeg();
    if (fabsf(normalizeAngle180(heading - s_still_heading_deg)) <= GESTURE_STILL_DEG) return false;
    s_still_heading_deg = heading;
    return true;
}

void restart() {
    s_input.reset(millis());
    s_mode = MODE_READY;
    s_still_heading_deg = g_imu.getHeadingDeg();
}

static void startBlinking(Mode mode, uint32_t now_ms) {
    s_mode = mode;
    s_blink_start_ms = now_ms;
    if (mode == MODE_STAGE) {
        Serial.printf("[UI] Stage: %s. A hand = next, leave it for %d blinks = do it.\n", kStageName[s_stage], (int)GESTURE_CONFIRM_BLINKS);
    } else {
        Serial.printf("[UI] Speed level %d of %d (%d%%). A hand = next level, leave it for %d blinks = go.\n",
                      (int)s_speed_level + 1, (int)SPEED_LEVELS, (int)kSpeedPercents[s_speed_level], (int)GESTURE_CONFIRM_BLINKS);
    }
}

// The blinks ran out with no hand: do what was being shown
static void confirm(uint32_t now_ms) {
    if (s_mode == MODE_SPEED) {
        Serial.printf("[UI] Confirmed: SPEED RUN at level %d (%d%%)\n", (int)s_speed_level + 1, (int)kSpeedPercents[s_speed_level]);
        Actions::selectMode(GESTURE_SPEEDRUN_MODE);
        Actions::setNextSpeedRunPercent(kSpeedPercents[s_speed_level]);
        Actions::launchSelectedRun();
    } else if (s_stage == STAGE_SPEED_RUN) {
        Serial.println("[UI] Confirmed: SPEED RUN. Now the speed.");
        startBlinking(MODE_SPEED, now_ms); // Nothing moves yet: the speed is chosen next, the same way
        return;
    } else if (s_stage == STAGE_SEARCH) {
        Serial.println("[UI] Confirmed: SEARCH");
        Actions::selectMode(Actions::MODE_SEARCH);
        Actions::launchSelectedRun();
    } else if (s_stage == STAGE_CALIBRATE) {
        Serial.println("[UI] Confirmed: RESET SENSORS + CALIBRATE");
        show(StatusLED::YELLOW, RGB_BRIGHTNESS_LEVEL, "CALIBRATING (measure, turn round, measure, turn back)");
        Serial.println(Actions::calibrateIRBothWays());
    } else {
        Serial.println("[UI] Confirmed: FORGET THE MAZE");
        Actions::clearSavedMaze();
    }
    restart(); // Back to ready, and learn again what the sensors see
}

void describe(char* buf, size_t size, const IRReadings& ir) {
    snprintf(buf, size, "HANDS: front sensors read %d now (FL=%d FR=%d); a hand is below %.0f or above %.0f. Hand %s. %s",
             (int)ir.front_center, (int)ir.front_left, (int)ir.front_right,
             fmaxf(0.0f, s_input.getDropLevel()), s_input.getHandLevel(),
             s_input.isHandPresent() ? "SEEN" : "not seen", s_led_meaning);
}

void describeForApp(char* meaning, size_t size, char color_hex[4], uint8_t& brightness) {
    StatusLED::Color color = s_led_color;
    brightness = s_led_brightness;
    if (Actions::isRunActive()) {
        // (the LED is then left in the run's colour by Actions, and hands are ignored)
        color = Actions::modeColor(Actions::getSelectedMode());
        brightness = RGB_BRIGHTNESS_LEVEL;
        snprintf(meaning, size, "RUN IN PROGRESS (%s): hands are ignored", Actions::modeName(Actions::getSelectedMode()));
    } else {
        strlcpy(meaning, s_led_meaning, size);
    }
    color_hex[0] = color.red   ? 'f' : '0';
    color_hex[1] = color.green ? 'f' : '0';
    color_hex[2] = color.blue  ? 'f' : '0';
    color_hex[3] = '\0';
}

void update(const IRReadings& ir) {
    static bool hand_was_present = false;
    const uint32_t now_ms = millis();
    char meaning[80];

    // A robot that is being carried, turned, or is driving a test move is not being talked to:
    // drop whatever was being chosen and wait until it has stood still again
    if (robotWasMoved() || !g_motion_controller.isCommandFinished()) {
        if (s_mode != MODE_READY) Serial.println("[UI] Robot moved: choosing cancelled, nothing was changed.");
        s_mode = MODE_READY;
        s_input.reset(now_ms);
    }

    const bool hand_came_and_went = s_input.update(ir.front_center, now_ms);

    // Say on the serial monitor what is being seen, so hands can be checked without the LED
    if (s_input.isHandPresent() != hand_was_present) {
        hand_was_present = s_input.isHandPresent();
        if (hand_was_present) Serial.printf("[UI] Hand seen (front sensors read %d)\n", (int)ir.front_center);
    }

    if (s_mode == MODE_READY) {
        if (hand_came_and_went) {
            s_stage = STAGE_SEARCH;
            startBlinking(MODE_STAGE, now_ms);
        } else if (s_input.isWarmingUp(now_ms)) {
            show(READY_COLOR, WAIT_BRIGHTNESS, "WAIT: settling, hands are ignored");
            return;
        } else {
            show(READY_COLOR, RGB_BRIGHTNESS_LEVEL, "READY: a hand at the front starts the choosing");
            return;
        }
    } else if (hand_came_and_went) {
        // A hand before the blinks ran out: the next one
        if (s_mode == MODE_SPEED) {
            s_speed_level = (s_speed_level + 1) % SPEED_LEVELS;
            startBlinking(MODE_SPEED, now_ms);
        } else if (s_stage + 1 < STAGE_COUNT) {
            s_stage++;
            startBlinking(MODE_STAGE, now_ms);
        } else {
            Serial.println("[UI] Past the last stage: back to ready, nothing chosen.");
            s_mode = MODE_READY;
            return;
        }
    }

    // While a hand is there the count waits: the blinks start again when it has gone
    if (s_input.isHandPresent()) s_blink_start_ms = now_ms;

    const uint32_t since_ms = now_ms - s_blink_start_ms;
    if (since_ms >= (uint32_t)GESTURE_CONFIRM_BLINKS * GESTURE_BLINK_MS) {
        confirm(now_ms);
        return;
    }

    const int blink = (int)(since_ms / GESTURE_BLINK_MS) + 1;
    const bool lit = s_input.isHandPresent() || (since_ms % GESTURE_BLINK_MS) < GESTURE_BLINK_MS / 2;
    if (s_mode == MODE_SPEED) {
        snprintf(meaning, sizeof(meaning), "SPEED level %d of %d (%d%%): blink %d of %d. Hand = next level",
                 (int)s_speed_level + 1, (int)SPEED_LEVELS, (int)kSpeedPercents[s_speed_level], blink, (int)GESTURE_CONFIRM_BLINKS);
        show(lit ? kStageColor[STAGE_SPEED_RUN] : StatusLED::OFF, kSpeedBrightness[s_speed_level], meaning);
    } else {
        snprintf(meaning, sizeof(meaning), "%s: blink %d of %d. Hand = next stage", kStageName[s_stage], blink, (int)GESTURE_CONFIRM_BLINKS);
        show(lit ? kStageColor[s_stage] : StatusLED::OFF, RGB_BRIGHTNESS_LEVEL, meaning);
    }
}

} // namespace GestureUI
