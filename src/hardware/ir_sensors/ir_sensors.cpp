#include "hardware/ir_sensors/ir_sensors.h"

// ==============================================================================
// IR WALL SENSORS
// ==============================================================================

IRSensors::IRSensors()
    : filtered_{0, 0, 0, 0, 0, 0},
      history_{},
      paused_(false),
      last_ambient_{},
      last_lit_{},
      update_count_(0),
      prev_l90_(0),
      prev_r90_(0),
      thresh_l90_(WALL_THRESH_L90),
      thresh_l45_(WALL_THRESH_L45),
      thresh_front_(WALL_THRESH_FRONT),
      front_level_measured_(false),
      nominal_fl_(0), nominal_fr_(0),
      thresh_r45_(WALL_THRESH_R45),
      thresh_r90_(WALL_THRESH_R90),
      nominal_center_l45_(NOMINAL_CENTER_L45),
      nominal_center_r45_(NOMINAL_CENTER_R45) {
    memset(&readings_, 0, sizeof(readings_));
}

// Channel order everywhere in this file: CH1..CH6 = L90, L45, FL, FR, R45, R90
static const uint8_t kEmitterPins[6]  = { PIN_IR_E1, PIN_IR_E2, PIN_IR_E3, PIN_IR_E4, PIN_IR_E5, PIN_IR_E6 };
static const uint8_t kReceiverPins[6] = { PIN_IR_R1, PIN_IR_R2, PIN_IR_R3, PIN_IR_R4, PIN_IR_R5, PIN_IR_R6 };

// Firing pairs (channel indices), one pair per control tick
static const uint8_t kFiringPairs[3][2] = {
    { 2, 3 }, // The two front sensors: FL, FR
    { 0, 5 }, // The two side sensors:  L90, R90
    { 1, 4 }  // The two diagonals:     L45, R45
};

void IRSensors::begin() {
    for (int i = 0; i < 6; ++i) {
        pinMode(kEmitterPins[i], OUTPUT);
        digitalWrite(kEmitterPins[i], LOW);
        pinMode(kReceiverPins[i], INPUT);
    }

    analogReadResolution(12);
    for (int i = 0; i < 6; ++i) {
        analogSetPinAttenuation(kReceiverPins[i], IR_ADC_ATTENUATION); // Sensitivity, see config.h
    }

    // Try loading previously saved calibration from NVS Flash
    if (!loadFromNVS()) {
        Serial.println("[IR] No stored calibration found in Flash; using defaults from config.h.");
    }
}

void IRSensors::sampleGroup(const uint8_t* channels, uint8_t count) {
    uint16_t ambient[6];

    // 1. Measure ambient light with the group's emitters OFF
    for (int i = 0; i < count; ++i) {
        ambient[i] = analogRead(kReceiverPins[channels[i]]);
    }

    // 2. Pulse the group's emitters ON together and let the phototransistors settle
    for (int i = 0; i < count; ++i) {
        digitalWrite(kEmitterPins[channels[i]], HIGH);
    }
    delayMicroseconds(IR_PULSE_SETTLE_US);

    // 3. Sample reflected signal, ambient-subtracted and low-pass filtered
    for (int i = 0; i < count; ++i) {
        const uint8_t ch = channels[i];
        uint16_t lit = analogRead(kReceiverPins[ch]);
        uint16_t raw = (lit > ambient[i]) ? (lit - ambient[i]) : 0;
        last_ambient_[ch] = ambient[i];
        last_lit_[ch] = lit;

        // Median of the last three samples throws away a single-sample spike (sunlight flicker,
        // a camera flash, electrical noise) without smearing real edges the way averaging would
        uint16_t a = raw, b = history_[ch][0], c = history_[ch][1];
        uint16_t median = max(min(a, b), min(max(a, b), c));
        history_[ch][1] = history_[ch][0];
        history_[ch][0] = raw;

        filtered_[ch] = (uint16_t)(IR_FILTER_ALPHA * median + (1.0f - IR_FILTER_ALPHA) * filtered_[ch]);
    }

    // 4. Emitters OFF
    for (int i = 0; i < count; ++i) {
        digitalWrite(kEmitterPins[channels[i]], LOW);
    }
}

// How far away a wall is, as a fraction of its distance when the robot is centred (1.0 = centred,
// below 1 = closer, above 1 = further). Reflected IR falls off with the square of distance, so
// distance goes as 1/sqrt(reading). Working in distance makes the steering error proportional to
// how far off-centre the robot actually is, instead of exploding as it nears a wall.
static float relativeDistance(uint16_t reading, uint16_t centred_reading) {
    float r = (reading > 1) ? (float)reading : 1.0f;
    return sqrtf((float)centred_reading / r);
}

void IRSensors::measureCrossTable(int16_t rise[6][6], uint32_t on_time_us) {
    paused_ = true;
    delay(6); // Let the control loop finish any pulse it had started

    for (int emitter = 0; emitter < 6; ++emitter) {
        int32_t sum[6] = {};
        const int repeats = 8;
        for (int r = 0; r < repeats; ++r) {
            uint16_t off[6];
            for (int rx = 0; rx < 6; ++rx) off[rx] = analogRead(kReceiverPins[rx]);
            digitalWrite(kEmitterPins[emitter], HIGH);
            delayMicroseconds(on_time_us);
            for (int rx = 0; rx < 6; ++rx) sum[rx] += (int32_t)analogRead(kReceiverPins[rx]) - (int32_t)off[rx];
            digitalWrite(kEmitterPins[emitter], LOW);
            delay(2);
        }
        for (int rx = 0; rx < 6; ++rx) rise[emitter][rx] = (int16_t)(sum[rx] / repeats);
    }

    paused_ = false;
}

void IRSensors::update() {
    if (paused_) return;
    sampleGroup(kFiringPairs[update_count_ % 3], 2);
    update_count_ = update_count_ + 1;

    readings_.left_90      = filtered_[0];
    readings_.left_45      = filtered_[1];
    readings_.front_left   = filtered_[2];
    readings_.front_right  = filtered_[3];
    readings_.right_45     = filtered_[4];
    readings_.right_90     = filtered_[5];
    readings_.front_center = (readings_.front_left > readings_.front_right) ? readings_.front_left : readings_.front_right;

    // Pillar / post edge detection (falling edge when passing a wall opening)
    readings_.post_edge_left  = (prev_l90_ > thresh_l90_) && (readings_.left_90 < thresh_l90_) && ((int32_t)prev_l90_ - readings_.left_90 > 60);
    readings_.post_edge_right = (prev_r90_ > thresh_r90_) && (readings_.right_90 < thresh_r90_) && ((int32_t)prev_r90_ - readings_.right_90 > 60);

    // Pillar / post rising edge (entering a wall from an opening)
    readings_.post_rising_left  = (prev_l90_ < thresh_l90_) && (readings_.left_90 > thresh_l90_) && ((int32_t)readings_.left_90 - prev_l90_ > 60);
    readings_.post_rising_right = (prev_r90_ < thresh_r90_) && (readings_.right_90 > thresh_r90_) && ((int32_t)readings_.right_90 - prev_r90_ > 60);

    prev_l90_ = readings_.left_90;
    prev_r90_ = readings_.right_90;

    // 90° sensors directly inspect current cell walls
    readings_.wall_left  = (readings_.left_90 > thresh_l90_);
    readings_.wall_right = (readings_.right_90 > thresh_r90_);
    readings_.wall_front = (readings_.front_center > thresh_front_);

    // Distance to a front wall, relative to the cell centre. The wall level is 40 % of what the
    // front sensors read at a cell centre during calibration, so that reading is level / 0.40.
    // The reading falls off with distance to the power FRONT_FALLOFF_EXPONENT, so
    // distance = centred distance * (centred / now) ^ (1 / exponent). (It used the square rule
    // here; on the robot that put the wall nearer than it was and every stop came up short.)
    readings_.front_offset_mm = 0.0f;
    if (front_level_measured_ && readings_.wall_front) {
        const float centred_reading = (float)thresh_front_ / 0.40f;
        const float now = (readings_.front_center > 1) ? (float)readings_.front_center : 1.0f;
        readings_.front_offset_mm = FRONT_SENSOR_TO_WALL_MM *
                                    (powf(centred_reading / now, 1.0f / FRONT_FALLOFF_EXPONENT) - 1.0f)
                                    - FRONT_STOP_EXTRA_MM; // Aim to stand this much further back than the centre
    }

    // Opening anticipation: 90° sensor detects wall present beside the robot,
    // but 45° lookahead sensor drops significantly below nominal, indicating the wall terminates ahead.
    const uint16_t opening_cutoff_l = (uint16_t)(nominal_center_l45_ * 0.65f);
    const uint16_t opening_cutoff_r = (uint16_t)(nominal_center_r45_ * 0.65f);

    readings_.opening_left  = readings_.wall_left  && (readings_.left_45  < opening_cutoff_l);
    readings_.opening_right = readings_.wall_right && (readings_.right_45 < opening_cutoff_r);

    // Determine reliable wall guides for steering
    bool valid_left_guide  = readings_.wall_left  && !readings_.opening_left  && (readings_.left_45  > thresh_l45_);
    bool valid_right_guide = readings_.wall_right && !readings_.opening_right && (readings_.right_45 > thresh_r45_);
    bool both_diagonals_funnel = (readings_.left_45 > thresh_l45_) && (readings_.right_45 > thresh_r45_);

    // What the steering measures the distance to each side wall with. The 90° sensors, when their
    // centred readings are known: they read 300-600 against a wall and move clearly with a few
    // millimetres of drift. The 45° sensors were used before; on this robot they read 110-170, as
    // much noise as signal, so the robot crept 8 mm toward a wall with the error still at zero,
    // and near a front wall they read THAT wall and swerved the robot (2026-10-10). They still
    // decide whether a side is a guide at all (wall there, and no opening coming up ahead).
    const bool steer_by_90 = ENABLE_SIDE_SENSOR_CENTERING && steer_by_side_sensors_ &&
                             nominal_center_l90_ > 0 && nominal_center_r90_ > 0;

    // A 45° sensor whose centred reading is small has little to steer by: a few counts of noise
    // are a large share of it. While the other side has a proper wall to follow, the weak side
    // is left out of the steering (it is still used when it is the only wall there is).
    // (Only while steering by the 45° sensors: the 90° ones are strong on both sides.)
    const bool weak_left  = nominal_center_l45_ < IR_CENTER_MIN_NOMINAL;
    const bool weak_right = nominal_center_r45_ < IR_CENTER_MIN_NOMINAL;
    if (!steer_by_90 && valid_left_guide && valid_right_guide && weak_left != weak_right) {
        if (weak_right) valid_right_guide = false;
        else            valid_left_guide = false;
    }

    const float guide_left  = steer_by_90 ? relativeDistance(readings_.left_90,  nominal_center_l90_)
                                          : relativeDistance(readings_.left_45,  nominal_center_l45_);
    const float guide_right = steer_by_90 ? relativeDistance(readings_.right_90, nominal_center_r90_)
                                          : relativeDistance(readings_.right_45, nominal_center_r45_);

    // Centering error calculation with opening anticipation & dynamic front-wall approach squaring
    const bool squaring_on_front = ENABLE_FRONT_SQUARING && readings_.wall_front;
    if (readings_.wall_front && !squaring_on_front) {
        // Front wall in view and squaring switched off: no wall steering at all, the heading is
        // held on the IMU. The side sensors cannot be used here, because close to a front wall
        // the 45° sensors are reading that wall, not the side walls.
        readings_.centering_error = 0.0f;
    } else if (squaring_on_front) {
        // Approaching a front wall: square up to it using the two front sensors. The left one
        // closer -> tilted -> positive error. (Each sensor is compared with its own calibrated
        // reading: see getFrontSkew(). Comparing the raw readings made the robot veer left at
        // every front wall, because the right sensor always reads higher.)
        readings_.centering_error = 1.5f * getFrontSkew();
    } else if (valid_left_guide && valid_right_guide) {
        // Both walls present: steer toward the side whose wall is further away
        readings_.centering_error = 2.0f * (guide_right - guide_left);
    } else if (both_diagonals_funnel && !readings_.wall_left && !readings_.wall_right) {
        // Re-entering a corridor with walls ahead on both sides: only the 45° sensors see them yet
        float dist_left  = relativeDistance(readings_.left_45,  nominal_center_l45_);
        float dist_right = relativeDistance(readings_.right_45, nominal_center_r45_);
        readings_.centering_error = 2.0f * (dist_right - dist_left);
    } else if (valid_left_guide) {
        // Right wall is opening or absent -> hold the centred distance from the left wall only
        readings_.centering_error = 4.0f * (1.0f - guide_left);
    } else if (valid_right_guide) {
        // Left wall is opening or absent -> hold the centred distance from the right wall only
        readings_.centering_error = 4.0f * (guide_right - 1.0f);
    } else {
        // Both walls open / open intersection -> maintain heading via IMU, zero steering bias
        readings_.centering_error = 0.0f;
    }

    // Tolerance: within IR_CENTER_TOLERANCE of the middle counts as centred, so the robot is not
    // forever steering after sensor noise. One unit of error is a quarter of the centred wall
    // distance, hence the 4. Beyond the tolerance only the excess is steered out, so the
    // correction starts gently instead of with a kick. Squaring up on a front wall is exempt.
    if (!squaring_on_front) {
        const float slack = 4.0f * IR_CENTER_TOLERANCE;
        if (readings_.centering_error > slack)       readings_.centering_error -= slack;
        else if (readings_.centering_error < -slack) readings_.centering_error += slack;
        else                                         readings_.centering_error = 0.0f;
    }

    // Clamp centering error to prevent extreme spikes from saturating actuators
    if (readings_.centering_error > 1.5f)  readings_.centering_error = 1.5f;
    if (readings_.centering_error < -1.5f) readings_.centering_error = -1.5f;
}

float IRSensors::getFrontSkew() const {
    if (nominal_fl_ < 50 || nominal_fr_ < 50) return 0.0f; // Not calibrated facing a wall: do not guess
    const float fl = (float)readings_.front_left  / (float)nominal_fl_;
    const float fr = (float)readings_.front_right / (float)nominal_fr_;
    const float average = (fl + fr) * 0.5f;
    return (average > 0.15f) ? (fl - fr) / average : 0.0f; // Too faint to compare = no opinion
}

bool IRSensors::calibrateInCell(uint16_t sample_count, bool keep_side_centre) {
    Serial.println("\n[CALIB] Starting In-Cell IR Auto-Calibration...");
    Serial.println("[CALIB] Ensure the mouse is placed squarely in the center of a cell with Left, Right, and Front walls!");

    uint32_t sum_l90 = 0, sum_l45 = 0, sum_fl = 0, sum_fr = 0, sum_r45 = 0, sum_r90 = 0;

    for (uint16_t i = 0; i < sample_count; ++i) {
        // The 500 Hz control loop owns the emitters; just average what it produces. If it is not
        // running (bench tests), sample both groups here instead.
        uint32_t seen = update_count_;
        delay(5);
        if (update_count_ == seen) {
            update();
            update();
            update();
        }
        sum_l90 += filtered_[0];
        sum_l45 += filtered_[1];
        sum_fl  += filtered_[2];
        sum_fr  += filtered_[3];
        sum_r45 += filtered_[4];
        sum_r90 += filtered_[5];
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

    // The 45° sensors point forward as well as sideways, so depending on where they are mounted
    // they may be looking at the side wall of the NEXT cell, which can have a gap in it. If one
    // of them sees nothing while the other does, borrow from the good side, scaled by how the two
    // 90° sensors compare (which both do see this cell's walls).
    // "Sees nothing" is judged against IR_CALIB_45_MIN, not a level a strong sensor would reach:
    // on this robot the right 45° sensor reads only 50-110 against a wall (the left one 230-420),
    // and with the old cut-off of 100 it was written off as "no wall" at most calibrations. Its
    // level was then guessed from the left sensor, three times too high, so a wall on the right
    // was never recognised and sometimes counted as an opening.
    // It must also be a fair share of what the other side would predict for it: a strong sensor
    // looking into a gap still reads something from the wall beyond, and that is not a wall.
    // (On the robot the weak right sensor comes to 23-33 % of the prediction; a gap is well below.)
    // Is there a wall right in front? Then the 45° sensors are reading THAT wall, not the side
    // walls, and what they read here must not become their "centred" value: calibrated facing the
    // start cell's back wall they came out 2-4 times too high (273 / 453 against 149 / 117 with
    // the way ahead clear), and in the corridor the wall steering then chased a reading it could
    // never reach and drove the robot into the left wall (2026-10-10). A front wall at a cell
    // centre reads about what the 90° sensors read from the side walls; an open way ahead reads
    // a twentieth of that.
    float side_90 = 0.0f;
    if (avg_l90 > 100 && avg_r90 > 100) side_90 = ((float)avg_l90 + (float)avg_r90) * 0.5f;
    else if (avg_l90 > 100)             side_90 = (float)avg_l90;
    else if (avg_r90 > 100)             side_90 = (float)avg_r90;
    const bool front_wall_here = (side_90 > 0.0f) ? ((float)avg_front >= 0.5f * side_90)
                                                  : ((float)avg_front >= ((float)avg_l45 + (float)avg_r45) * 0.75f);
    last_calibration_had_front_wall_ = front_wall_here;
    const bool keep_45 = front_wall_here && nominal_center_l45_ > 0 && nominal_center_r45_ > 0;
    if (keep_45) {
        Serial.printf("[CALIB] Wall right in front: the 45° sensors are reading it, so their centred readings are kept as they were (%d / %d).\n",
                      nominal_center_l45_, nominal_center_r45_);
        avg_l45 = nominal_center_l45_;
        avg_r45 = nominal_center_r45_;
    } else if (front_wall_here) {
        Serial.println("[CALIB] WARNING: wall right in front and no earlier 45° readings to keep. Calibrate again facing along a corridor.");
    }

    const float left_to_right = (avg_l90 > 100 && avg_r90 > 100) ? (float)avg_r90 / (float)avg_l90 : 1.0f;
    const bool left_ok  = avg_l45 >= IR_CALIB_45_MIN && (float)avg_l45 >= IR_CALIB_45_MIN_SHARE * (float)avg_r45 / left_to_right;
    const bool right_ok = avg_r45 >= IR_CALIB_45_MIN && (float)avg_r45 >= IR_CALIB_45_MIN_SHARE * (float)avg_l45 * left_to_right;
    if (!left_ok && !right_ok) {
        Serial.println("[CALIB] ERROR: Side sensor readings too low! Ensure robot is centered between left/right walls.");
        return false;
    }
    if (!left_ok || !right_ok) {
        if (!right_ok) avg_r45 = (uint16_t)((float)avg_l45 * left_to_right);
        if (!left_ok)  avg_l45 = (uint16_t)((float)avg_r45 / left_to_right);
        Serial.println("[CALIB] Note: one 45° sensor saw no wall (gap ahead on that side). Estimated it from the other side.");
    }

    // Set nominal center values for steering
    nominal_center_l45_ = avg_l45;
    nominal_center_r45_ = avg_r45;
    // The 90° sensors' centred readings, for the wall steering. They look straight sideways, so a
    // wall in front does not disturb them; what matters is that the robot is standing where it
    // was centred by hand, which it no longer is after turning round (hence keep_side_centre).
    if (!keep_side_centre || nominal_center_l90_ == 0 || nominal_center_r90_ == 0) {
        if (avg_l90 > 100) nominal_center_l90_ = avg_l90;
        if (avg_r90 > 100) nominal_center_r90_ = avg_r90;
    }
    Serial.printf("[CALIB] 90° sensors: centred reading left %d, right %d%s.\n", nominal_center_l90_, nominal_center_r90_,
                  keep_side_centre ? " (kept from the first measurement)" : "");

    // Detection thresholds set to 40% of nominal wall reflection
    thresh_l90_ = (avg_l90 > 100) ? (uint16_t)(avg_l90 * 0.40f) : WALL_THRESH_L90;
    // (a weak sensor's 40 % would sit down among what it reads with no wall at all, hence the floor)
    thresh_l45_ = (uint16_t)fmaxf(avg_l45 * 0.40f, IR_45_MIN_WALL_LEVEL);
    thresh_r45_ = (uint16_t)fmaxf(avg_r45 * 0.40f, IR_45_MIN_WALL_LEVEL);
    Serial.printf("[CALIB] 45° sensors: centred reading left %d, right %d -> 'wall' above %d / %d.\n",
                  avg_l45, avg_r45, thresh_l45_, thresh_r45_);
    thresh_r90_ = (avg_r90 > 100) ? (uint16_t)(avg_r90 * 0.40f) : WALL_THRESH_R90;

    // Is there a wall right in front? One this close reads far brighter than the side walls do
    // on the 45° sensors. A dimmer reading is only a wall further down an open corridor, and must
    // not be used as the "wall in front" level (it would make every distant wall look close).
    const float side_level = ((float)avg_l45 + (float)avg_r45) * 0.5f;
    if (front_wall_here) {
        thresh_front_ = (uint16_t)(avg_front * 0.40f);
        front_level_measured_ = true;
        nominal_fl_ = avg_fl; // The robot is square to this wall, so this is how the two compare
        nominal_fr_ = avg_fr;
        Serial.printf("[CALIB] Front wall measured: reads %d here, so 'wall in front' = above %d.\n", avg_front, thresh_front_);
    } else if (front_level_measured_) {
        // No wall in front this time (the start cell usually opens into the maze). The level
        // measured against a real wall is far better than any estimate, so it is kept.
        Serial.printf("[CALIB] No wall in front here: keeping the front level measured earlier (%d).\n", thresh_front_);
    } else {
        // Never measured against a real wall, so it has to be estimated. A side wall is the same
        // distance from a 90° sensor as a front wall is from the front sensors at a cell centre,
        // and both look at it square-on, so the 90° readings are the best stand-in. (The 45°
        // sensors used before read much lower, which put the level so low that a wall a whole
        // cell further away counted as "right in front".)
        float from_45 = side_level * 0.45f;
        float from_90 = 0.0f;
        if (avg_l90 > 100 && avg_r90 > 100) from_90 = ((float)avg_l90 + (float)avg_r90) * 0.5f * 0.40f;
        else if (avg_l90 > 100)             from_90 = (float)avg_l90 * 0.40f;
        else if (avg_r90 > 100)             from_90 = (float)avg_r90 * 0.40f;
        thresh_front_ = (uint16_t)((from_90 > from_45) ? from_90 : from_45);
        Serial.printf("[CALIB] WARNING: no wall in front, and the front level has never been measured. Guessing %d from the side sensors.\n",
                      thresh_front_);
        Serial.println("[CALIB]          To measure it: put the robot in the middle of a cell FACING A WALL (walls on both sides too) and calibrate once.");
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
    prefs_.putBool("f_meas",  front_level_measured_);
    prefs_.putUShort("fl_c",  nominal_fl_);
    prefs_.putUShort("fr_c",  nominal_fr_);
    prefs_.putUShort("r45_t", thresh_r45_);
    prefs_.putUShort("r90_t", thresh_r90_);
    prefs_.putUShort("l45_c", nominal_center_l45_);
    prefs_.putUShort("r45_c", nominal_center_r45_);
    prefs_.putUShort("l90_c", nominal_center_l90_);
    prefs_.putUShort("r90_c", nominal_center_r90_);
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
    front_level_measured_ = prefs_.getBool("f_meas", false); // Levels saved by older firmware count as estimates
    nominal_fl_         = prefs_.getUShort("fl_c", 0);
    nominal_fr_         = prefs_.getUShort("fr_c", 0);
    thresh_r45_         = prefs_.getUShort("r45_t", WALL_THRESH_R45);
    thresh_r90_         = prefs_.getUShort("r90_t", WALL_THRESH_R90);
    nominal_center_l45_ = prefs_.getUShort("l45_c", NOMINAL_CENTER_L45);
    nominal_center_r45_ = prefs_.getUShort("r45_c", NOMINAL_CENTER_R45);
    nominal_center_l90_ = prefs_.getUShort("l90_c", 0); // 0 = never measured: steering falls back to the 45° sensors
    nominal_center_r90_ = prefs_.getUShort("r90_c", 0);
    prefs_.end();

    Serial.printf("[IR] Loaded from Flash: CenterL45=%d, CenterR45=%d, FrontThresh=%d (%s)\n",
                  nominal_center_l45_, nominal_center_r45_, thresh_front_,
                  front_level_measured_ ? "measured against a wall" : "only an estimate");
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

bool IRSensors::hasLeftPostRising() const {
    return readings_.post_rising_left;
}

bool IRSensors::hasRightPostRising() const {
    return readings_.post_rising_right;
}

bool IRSensors::hasLeftOpening() const {
    return readings_.opening_left;
}

bool IRSensors::hasRightOpening() const {
    return readings_.opening_right;
}

float IRSensors::getCenteringError() const {
    return readings_.centering_error;
}

float IRSensors::getDiagonalGuardError() const {
    // Each 45° sensor starts to complain once it reads DIAG_GUARD_RATIO times its normal
    // "centred in a corridor" level, and is fully alarmed at twice that
    float guard_l = (float)nominal_center_l45_ * DIAG_GUARD_RATIO;
    float guard_r = (float)nominal_center_r45_ * DIAG_GUARD_RATIO;
    float excess_l = ((float)readings_.left_45  - guard_l) / guard_l;
    float excess_r = ((float)readings_.right_45 - guard_r) / guard_r;
    if (excess_l < 0.0f) excess_l = 0.0f;
    if (excess_r < 0.0f) excess_r = 0.0f;
    return constrain(excess_l - excess_r, -1.0f, 1.0f);
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
