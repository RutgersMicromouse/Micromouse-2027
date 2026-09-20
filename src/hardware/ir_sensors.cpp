#include "ir_sensors.h"

IRSensors::IRSensors()
    : prev_l90_(0),
      prev_r90_(0),
      thresh_l90_(WALL_THRESH_L90),
      thresh_l45_(WALL_THRESH_L45),
      thresh_front_(WALL_THRESH_FRONT),
      thresh_r45_(WALL_THRESH_R45),
      thresh_r90_(WALL_THRESH_R90),
      nominal_center_l45_(NOMINAL_CENTER_L45),
      nominal_center_r45_(NOMINAL_CENTER_R45) {
    memset(&readings_, 0, sizeof(readings_));
}

void IRSensors::begin() {
    const uint8_t emitter_pins[6] = {
        PIN_IR_E1, PIN_IR_E2, PIN_IR_E3, PIN_IR_E4, PIN_IR_E5, PIN_IR_E6
    };
    for (int i = 0; i < 6; ++i) {
        pinMode(emitter_pins[i], OUTPUT);
        digitalWrite(emitter_pins[i], LOW);
    }

    pinMode(PIN_IR_R1, INPUT);
    pinMode(PIN_IR_R2, INPUT);
    pinMode(PIN_IR_R3, INPUT);
    pinMode(PIN_IR_R4, INPUT);
    pinMode(PIN_IR_R5, INPUT);
    pinMode(PIN_IR_R6, INPUT);

    analogReadResolution(12);

    // Try loading previously saved calibration from NVS Flash
    if (!loadFromNVS()) {
        Serial.println("[IR] No stored calibration found in Flash; using defaults from config.h.");
    }
}

uint16_t IRSensors::readPulsedChannel(uint8_t emitter_pin, uint8_t receiver_pin) {
    // 1. Measure ambient light with emitter OFF
    uint16_t ambient = analogRead(receiver_pin);

    // 2. Pulse emitter ON
    digitalWrite(emitter_pin, HIGH);
    delayMicroseconds(IR_PULSE_SETTLE_US);

    // 3. Sample reflected signal
    uint16_t signal = analogRead(receiver_pin);

    // 4. Turn emitter OFF
    digitalWrite(emitter_pin, LOW);

    // 5. Ambient-subtracted value
    if (signal > ambient) {
        return (signal - ambient);
    }
    return 0;
}

void IRSensors::update() {
    uint16_t raw_l90 = readPulsedChannel(PIN_IR_E1, PIN_IR_R1); // Pair 1: Left 90°
    uint16_t raw_l45 = readPulsedChannel(PIN_IR_E2, PIN_IR_R2); // Pair 2: Front-Left 45°
    uint16_t raw_fl  = readPulsedChannel(PIN_IR_E3, PIN_IR_R3); // Pair 3: Front-Left Center
    uint16_t raw_fr  = readPulsedChannel(PIN_IR_E4, PIN_IR_R4); // Pair 4: Front-Right Center
    uint16_t raw_r45 = readPulsedChannel(PIN_IR_E5, PIN_IR_R5); // Pair 5: Front-Right 45°
    uint16_t raw_r90 = readPulsedChannel(PIN_IR_E6, PIN_IR_R6); // Pair 6: Right 90°

    const float alpha = 0.5f;
    readings_.left_90      = (uint16_t)(alpha * raw_l90 + (1.0f - alpha) * readings_.left_90);
    readings_.left_45      = (uint16_t)(alpha * raw_l45 + (1.0f - alpha) * readings_.left_45);
    readings_.front_left   = (uint16_t)(alpha * raw_fl  + (1.0f - alpha) * readings_.front_left);
    readings_.front_right  = (uint16_t)(alpha * raw_fr  + (1.0f - alpha) * readings_.front_right);
    readings_.front_center = (readings_.front_left > readings_.front_right) ? readings_.front_left : readings_.front_right;
    readings_.right_45     = (uint16_t)(alpha * raw_r45 + (1.0f - alpha) * readings_.right_45);
    readings_.right_90     = (uint16_t)(alpha * raw_r90 + (1.0f - alpha) * readings_.right_90);

    // Pillar / post edge detection (falling edge when passing a wall opening)
    readings_.post_edge_left  = (prev_l90_ > thresh_l90_) && (readings_.left_90 < thresh_l90_) && ((int32_t)prev_l90_ - readings_.left_90 > 60);
    readings_.post_edge_right = (prev_r90_ > thresh_r90_) && (readings_.right_90 < thresh_r90_) && ((int32_t)prev_r90_ - readings_.right_90 > 60);

    prev_l90_ = readings_.left_90;
    prev_r90_ = readings_.right_90;

    // 90° sensors directly inspect current cell walls
    readings_.wall_left  = (readings_.left_90 > thresh_l90_);
    readings_.wall_right = (readings_.right_90 > thresh_r90_);
    readings_.wall_front = (readings_.front_center > thresh_front_);

    // Centering error calculation with front-wall suppression
    bool has_both_diagonals = (readings_.left_45 > thresh_l45_) && (readings_.right_45 > thresh_r45_);

    if (readings_.wall_front) {
        // Suppress 45° steering trim when approaching a front wall
        readings_.centering_error = 0.0f;
    } else if (has_both_diagonals) {
        float err_left  = (float)readings_.left_45  - (float)nominal_center_l45_;
        float err_right = (float)readings_.right_45 - (float)nominal_center_r45_;
        readings_.centering_error = (err_left - err_right) / (float)nominal_center_l45_;
    } else if (readings_.left_45 > thresh_l45_ && readings_.wall_left) {
        readings_.centering_error = 2.0f * ((float)readings_.left_45 - (float)nominal_center_l45_) / (float)nominal_center_l45_;
    } else if (readings_.right_45 > thresh_r45_ && readings_.wall_right) {
        readings_.centering_error = -2.0f * ((float)readings_.right_45 - (float)nominal_center_r45_) / (float)nominal_center_r45_;
    } else {
        readings_.centering_error = 0.0f;
    }
}

bool IRSensors::calibrateInCell(uint16_t sample_count) {
    Serial.println("\n[CALIB] Starting In-Cell IR Auto-Calibration...");
    Serial.println("[CALIB] Ensure the mouse is placed squarely in the center of a cell with Left, Right, and Front walls!");

    uint32_t sum_l90 = 0, sum_l45 = 0, sum_fl = 0, sum_fr = 0, sum_r45 = 0, sum_r90 = 0;

    for (uint16_t i = 0; i < sample_count; ++i) {
        sum_l90 += readPulsedChannel(PIN_IR_E1, PIN_IR_R1);
        sum_l45 += readPulsedChannel(PIN_IR_E2, PIN_IR_R2);
        sum_fl  += readPulsedChannel(PIN_IR_E3, PIN_IR_R3);
        sum_fr  += readPulsedChannel(PIN_IR_E4, PIN_IR_R4);
        sum_r45 += readPulsedChannel(PIN_IR_E5, PIN_IR_R5);
        sum_r90 += readPulsedChannel(PIN_IR_E6, PIN_IR_R6);
        delay(5);
    }

    uint16_t avg_l90   = sum_l90 / sample_count;
    uint16_t avg_l45   = sum_l45 / sample_count;
    uint16_t avg_fl    = sum_fl  / sample_count;
    uint16_t avg_fr    = sum_fr  / sample_count;
    uint16_t avg_front = (avg_fl > avg_fr) ? avg_fl : avg_fr;
    uint16_t avg_r45   = sum_r45 / sample_count;
    uint16_t avg_r90   = sum_r90 / sample_count;

    Serial.printf("[CALIB] Measured Cell Averages: L90=%d, L45=%d, FL=%d, FR=%d, R45=%d, R90=%d\n",
                  avg_l90, avg_l45, avg_fl, avg_fr, avg_r45, avg_r90);

    // Sanity check: Ensure side walls are present for centering baseline
    if (avg_l45 < 100 || avg_r45 < 100) {
        Serial.println("[CALIB] ERROR: Side sensor readings too low! Ensure robot is centered between left/right walls.");
        return false;
    }

    // Set nominal center values for steering
    nominal_center_l45_ = avg_l45;
    nominal_center_r45_ = avg_r45;

    // Detection thresholds set to 40% of nominal wall reflection
    thresh_l90_ = (avg_l90 > 100) ? (uint16_t)(avg_l90 * 0.40f) : WALL_THRESH_L90;
    thresh_l45_ = (uint16_t)(avg_l45 * 0.40f);
    thresh_r45_ = (uint16_t)(avg_r45 * 0.40f);
    thresh_r90_ = (avg_r90 > 100) ? (uint16_t)(avg_r90 * 0.40f) : WALL_THRESH_R90;

    // If front wall is present, calibrate from measurement; otherwise estimate from side 45° sensors
    if (avg_front >= 100) {
        thresh_front_ = (uint16_t)(avg_front * 0.40f);
    } else {
        Serial.println("[CALIB] Note: Front wall absent (open start corridor). Estimating front threshold from side sensors.");
        thresh_front_ = (uint16_t)(((float)avg_l45 + (float)avg_r45) * 0.5f * 0.45f);
    }

    // Save permanently to NVS Flash
    saveToNVS();
    Serial.println("[CALIB] Calibration successfully saved to Flash NVS!\n");
    return true;
}

void IRSensors::saveToNVS() {
    prefs_.begin("ir_calib", false); // Namespace "ir_calib", read/write
    prefs_.putUShort("l90_t", thresh_l90_);
    prefs_.putUShort("l45_t", thresh_l45_);
    prefs_.putUShort("f_t",   thresh_front_);
    prefs_.putUShort("r45_t", thresh_r45_);
    prefs_.putUShort("r90_t", thresh_r90_);
    prefs_.putUShort("l45_c", nominal_center_l45_);
    prefs_.putUShort("r45_c", nominal_center_r45_);
    prefs_.putBool("valid", true);
    prefs_.end();
}

bool IRSensors::loadFromNVS() {
    prefs_.begin("ir_calib", true); // Read-only
    if (!prefs_.getBool("valid", false)) {
        prefs_.end();
        return false;
    }

    thresh_l90_         = prefs_.getUShort("l90_t", WALL_THRESH_L90);
    thresh_l45_         = prefs_.getUShort("l45_t", WALL_THRESH_L45);
    thresh_front_       = prefs_.getUShort("f_t",   WALL_THRESH_FRONT);
    thresh_r45_         = prefs_.getUShort("r45_t", WALL_THRESH_R45);
    thresh_r90_         = prefs_.getUShort("r90_t", WALL_THRESH_R90);
    nominal_center_l45_ = prefs_.getUShort("l45_c", NOMINAL_CENTER_L45);
    nominal_center_r45_ = prefs_.getUShort("r45_c", NOMINAL_CENTER_R45);
    prefs_.end();

    Serial.printf("[IR] Loaded from Flash: CenterL45=%d, CenterR45=%d, FrontThresh=%d\n",
                  nominal_center_l45_, nominal_center_r45_, thresh_front_);
    return true;
}

IRReadings IRSensors::getReadings() const {
    return readings_;
}

bool IRSensors::hasLeftWall() const {
    return readings_.wall_left;
}

bool IRSensors::hasRightWall() const {
    return readings_.wall_right;
}

bool IRSensors::hasFrontWall() const {
    return readings_.wall_front;
}

bool IRSensors::hasLeftPostEdge() const {
    return readings_.post_edge_left;
}

bool IRSensors::hasRightPostEdge() const {
    return readings_.post_edge_right;
}

float IRSensors::getCenteringError() const {
    return readings_.centering_error;
}

void IRSensors::setThresholds(uint16_t thresh_l90, uint16_t thresh_l45,
                              uint16_t thresh_front,
                              uint16_t thresh_r45, uint16_t thresh_r90) {
    thresh_l90_   = thresh_l90;
    thresh_l45_   = thresh_l45;
    thresh_front_ = thresh_front;
    thresh_r45_   = thresh_r45;
    thresh_r90_   = thresh_r90;
}

void IRSensors::setNominalCenters(uint16_t center_l45, uint16_t center_r45) {
    nominal_center_l45_ = center_l45;
    nominal_center_r45_ = center_r45;
}
