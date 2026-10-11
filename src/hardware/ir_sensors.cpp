#include "ir_sensors.h"

IRSensorArray ir_sensors;

IRSensorArray::IRSensorArray()
    : thresh_front_(IR_WALL_DETECT_FRONT),
      thresh_side_(IR_WALL_DETECT_SIDE),
      nominal_side_dist_mm_(NOMINAL_SIDE_WALL_MM),
      prev_left_side_(0),
      prev_right_side_(0),
      post_rising_l_(false),
      post_falling_l_(false),
      post_rising_r_(false),
      post_falling_r_(false),
      filter_initialized_(false) {
    memset(&readings_, 0, sizeof(readings_));
}

void IRSensorArray::begin() {
    // Thresholds below are specified for a 10-bit ADC scale.
    analogReadResolution(10);
    analogReadAveraging(4);
    pinMode(PIN_IR_FRONT, INPUT);
    pinMode(PIN_IR_FRONT_LEFT, INPUT);
    pinMode(PIN_IR_REAR_LEFT, INPUT);
    pinMode(PIN_IR_FRONT_RIGHT, INPUT);
    pinMode(PIN_IR_REAR_RIGHT, INPUT);

    // Initial warm-up read
    update();
    Serial.println("[IR] Front sensor and four parallel side sensors initialized.");
}

float IRSensorArray::adcToVoltage(uint16_t raw_adc) {
    return ((float)raw_adc / ADC_RESOLUTION) * ADC_REF_VOLTAGE;
}

float IRSensorArray::voltageToDistanceMM(float voltage) {
    // Sharp response model: distance (mm) = A / (voltage - B).
    if (voltage <= (SHARP_0A51SK_B + 0.05f)) {
        return SHARP_MAX_DIST_MM;
    }
    float distance = SHARP_0A51SK_A / (voltage - SHARP_0A51SK_B);
    return constrain(distance, SHARP_MIN_DIST_MM, SHARP_MAX_DIST_MM);
}

uint16_t IRSensorArray::readAnalogOversampled(uint8_t pin, uint8_t samples) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < samples; ++i) {
        sum += analogRead(pin);
    }
    return (uint16_t)(sum / samples);
}

void IRSensorArray::update() {
    // 1. Read raw analog values with oversampling
    uint16_t raw_front = readAnalogOversampled(PIN_IR_FRONT);
    uint16_t raw_front_left = readAnalogOversampled(PIN_IR_FRONT_LEFT);
    uint16_t raw_rear_left = readAnalogOversampled(PIN_IR_REAR_LEFT);
    uint16_t raw_front_right = readAnalogOversampled(PIN_IR_FRONT_RIGHT);
    uint16_t raw_rear_right = readAnalogOversampled(PIN_IR_REAR_RIGHT);

    // 2. Exponential moving average low-pass filter.  Seed it from the first
    // physical sample so the first decision uses a real sensor value.
    const float alpha = 0.5f;
    if (!filter_initialized_) {
        readings_.front = raw_front;
        readings_.front_left = raw_front_left;
        readings_.rear_left = raw_rear_left;
        readings_.front_right = raw_front_right;
        readings_.rear_right = raw_rear_right;
        filter_initialized_ = true;
    } else {
        readings_.front = (uint16_t)(alpha * raw_front + (1.0f - alpha) * readings_.front);
        readings_.front_left = (uint16_t)(alpha * raw_front_left + (1.0f - alpha) * readings_.front_left);
        readings_.rear_left = (uint16_t)(alpha * raw_rear_left + (1.0f - alpha) * readings_.rear_left);
        readings_.front_right = (uint16_t)(alpha * raw_front_right + (1.0f - alpha) * readings_.front_right);
        readings_.rear_right = (uint16_t)(alpha * raw_rear_right + (1.0f - alpha) * readings_.rear_right);
    }

    // 3. Compute physical distances in millimeters via Sharp 0A51SK curve
    readings_.front_mm    = voltageToDistanceMM(adcToVoltage(readings_.front));
    readings_.front_left_mm = voltageToDistanceMM(adcToVoltage(readings_.front_left));
    readings_.rear_left_mm = voltageToDistanceMM(adcToVoltage(readings_.rear_left));
    readings_.front_right_mm = voltageToDistanceMM(adcToVoltage(readings_.front_right));
    readings_.rear_right_mm = voltageToDistanceMM(adcToVoltage(readings_.rear_right));

    // 4. Binary wall presence classification
    readings_.wall_front = readings_.front >= thresh_front_;

    // Continuous side wall presence vs. opening classification:
    // In a micromouse cell, a continuous side wall spans 168 mm along the entire cell.
    // When centered or stationary in the cell, a real side wall illuminates BOTH the front-side
    // and rear-side sensors (~49 mm standoff, nominal ~325 ADC).
    // An opening to an adjacent cell gives open air (> 180 mm distance, ADC < 130).
    // If either sensor sees open air (< peg_floor_adc), this side is an OPENING, NOT a wall!
    // Requiring both sensors to confirm presence above peg_floor_adc (and at least one above thresh_side_)
    // rejects corner pegs and previous/future adjacent walls.
    const uint16_t peg_floor_adc = (thresh_side_ > 35) ? (thresh_side_ - 35) : 155;
    readings_.wall_left  = (readings_.front_left >= thresh_side_ && readings_.rear_left >= peg_floor_adc) ||
                           (readings_.rear_left >= thresh_side_ && readings_.front_left >= peg_floor_adc);
    readings_.wall_right = (readings_.front_right >= thresh_side_ && readings_.rear_right >= peg_floor_adc) ||
                           (readings_.rear_right >= thresh_side_ && readings_.front_right >= peg_floor_adc);

    // 5. Detect side-wall transitions using the mean signal from each pair.
    uint16_t left_side_signal = (readings_.front_left + readings_.rear_left) / 2;
    uint16_t right_side_signal = (readings_.front_right + readings_.rear_right) / 2;
    int16_t delta_left = (int16_t)left_side_signal - (int16_t)prev_left_side_;
    int16_t delta_right = (int16_t)right_side_signal - (int16_t)prev_right_side_;

    post_rising_l_  = (delta_left > 70);
    post_falling_l_ = (delta_left < -70);
    post_rising_r_  = (delta_right > 70);
    post_falling_r_ = (delta_right < -70);

    prev_left_side_ = left_side_signal;
    prev_right_side_ = right_side_signal;

    // 6. Estimate lateral offset from paired side-wall distances.
    const uint16_t guide_floor_adc = (thresh_side_ > 35) ? (thresh_side_ - 35) : 155;

    // Both front and rear sensors must confirm wall presence to reject isolated corner posts.
    // Requiring at least one sensor >= thresh_side_ and the other >= guide_floor_adc accommodates
    // slight chassis yaw angles without losing wall tracking.
    bool valid_left_guide = ((readings_.front_left >= thresh_side_ && readings_.rear_left >= guide_floor_adc) ||
                             (readings_.rear_left >= thresh_side_ && readings_.front_left >= guide_floor_adc)) &&
                            readings_.front_left_mm < WALL_DETECT_DIST_MM &&
                            readings_.rear_left_mm < WALL_DETECT_DIST_MM;

    bool valid_right_guide = ((readings_.front_right >= thresh_side_ && readings_.rear_right >= guide_floor_adc) ||
                              (readings_.rear_right >= thresh_side_ && readings_.front_right >= guide_floor_adc)) &&
                             readings_.front_right_mm < WALL_DETECT_DIST_MM &&
                             readings_.rear_right_mm < WALL_DETECT_DIST_MM;

    float left_dist = 0.5f * (readings_.front_left_mm + readings_.rear_left_mm);
    float right_dist = 0.5f * (readings_.front_right_mm + readings_.rear_right_mm);

    float ref_dist = (nominal_side_dist_mm_ >= 38.0f && nominal_side_dist_mm_ <= 60.0f)
                     ? nominal_side_dist_mm_ : NOMINAL_SIDE_WALL_MM;

    if (valid_left_guide && valid_right_guide) {
        // Dual-wall centering: balance lateral distance between left and right walls
        float dist_diff = right_dist - left_dist;
        // Narrow deadband to +/- 1.2 mm so robot centers tightly in corridor
        if (fabsf(dist_diff) < 1.2f) {
            readings_.centering_error = 0.0f;
        } else if (dist_diff > 1.2f) {
            readings_.centering_error = constrain((dist_diff - 1.2f) / 12.0f, 0.0f, 1.0f);
        } else {
            readings_.centering_error = constrain((dist_diff + 1.2f) / 12.0f, -1.0f, 0.0f);
        }

        // Dynamically track nominal center distance in symmetric corridors
        float current_nominal = 0.5f * (left_dist + right_dist);
        if (current_nominal >= 40.0f && current_nominal <= 58.0f) {
            nominal_side_dist_mm_ = 0.98f * nominal_side_dist_mm_ + 0.02f * current_nominal;
        }
    } else if (valid_left_guide && !valid_right_guide) {
        // Single left wall: maintain nominal standoff (ref_dist ~49mm) from left wall.
        // If left_dist < ref_dist (too close to left): error > 0 -> positive centering_error -> steers RIGHT.
        // If left_dist > ref_dist (too far from left): error < 0 -> negative centering_error -> steers LEFT.
        if (left_dist < 80.0f) {
            float err_left = ref_dist - left_dist;
            if (fabsf(err_left) < 1.0f) {
                readings_.centering_error = 0.0f;
            } else if (err_left > 1.0f) {
                readings_.centering_error = constrain((err_left - 1.0f) / 7.0f, 0.0f, 1.0f);
            } else {
                readings_.centering_error = constrain((err_left + 1.0f) / 7.0f, -1.0f, 0.0f);
            }
        } else {
            readings_.centering_error = 0.0f;
        }
    } else if (valid_right_guide && !valid_left_guide) {
        // Single right wall: maintain nominal standoff (ref_dist ~49mm) from right wall.
        // If right_dist < ref_dist (too close to right): error < 0 -> negative centering_error -> steers LEFT.
        // If right_dist > ref_dist (too far from right): error > 0 -> positive centering_error -> steers RIGHT.
        if (right_dist < 80.0f) {
            float err_right = right_dist - ref_dist;
            if (fabsf(err_right) < 1.0f) {
                readings_.centering_error = 0.0f;
            } else if (err_right > 1.0f) {
                readings_.centering_error = constrain((err_right - 1.0f) / 7.0f, 0.0f, 1.0f);
            } else {
                readings_.centering_error = constrain((err_right + 1.0f) / 7.0f, -1.0f, 0.0f);
            }
        } else {
            readings_.centering_error = 0.0f;
        }
    } else {
        readings_.centering_error = 0.0f;
    }

    // Safety proximity repulsion:
    // Only intervene if chassis is dangerously close to scraping a wall (< 36 mm).
    // Nominal centered standoff is 49 mm. A 13 mm buffer prevents phantom steering at openings.
    float min_left = 999.0f;
    if (readings_.front_left >= thresh_side_) min_left = fminf(min_left, readings_.front_left_mm);
    if (readings_.rear_left >= thresh_side_)  min_left = fminf(min_left, readings_.rear_left_mm);
    if (min_left < 36.0f) {
        float repulse_right = (36.0f - min_left) / 8.0f; // Gently steer away from wall
        if (repulse_right > readings_.centering_error) {
            readings_.centering_error = constrain(repulse_right, 0.0f, 1.0f);
        }
    }

    float min_right = 999.0f;
    if (readings_.front_right >= thresh_side_) min_right = fminf(min_right, readings_.front_right_mm);
    if (readings_.rear_right >= thresh_side_)  min_right = fminf(min_right, readings_.rear_right_mm);
    if (min_right < 36.0f) {
        float repulse_left = (36.0f - min_right) / 8.0f; // Gently steer away from wall
        if (-repulse_left < readings_.centering_error) {
            readings_.centering_error = constrain(-repulse_left, -1.0f, 0.0f);
        }
    }

    readings_.centering_error = constrain(readings_.centering_error, -1.0f, 1.0f);

    // A yawed robot sees different distances at the front and rear sensors.
    float alignment_error = 0.0f;
    uint8_t alignment_sides = 0;
    if (valid_left_guide) {
        alignment_error += atan2f(readings_.front_left_mm - readings_.rear_left_mm,
                                  SIDE_SENSOR_SPACING_MM) * (180.0f / 3.1415926535f);
        ++alignment_sides;
    }
    if (valid_right_guide) {
        alignment_error -= atan2f(readings_.front_right_mm - readings_.rear_right_mm,
                                  SIDE_SENSOR_SPACING_MM) * (180.0f / 3.1415926535f);
        ++alignment_sides;
    }
    readings_.wall_alignment_error_deg = alignment_sides > 0
        ? constrain(alignment_error / alignment_sides, -15.0f, 15.0f)
        : 0.0f;
}

void IRSensorArray::flushFilter(uint8_t count) {
    filter_initialized_ = false;
    for (uint8_t i = 0; i < count; ++i) {
        update();
        delayMicroseconds(500);
    }
}

bool IRSensorArray::calibrateInCell(uint16_t sample_count) {
    if (sample_count == 0) {
        Serial.println("[IR] Calibration requires at least one sample.");
        return false;
    }

    Serial.println("[IR] Calibrating Sharp GP2Y0A51SK0F Sensors in cell...");

    uint32_t sum_front_left = 0, sum_rear_left = 0;
    uint32_t sum_front_right = 0, sum_rear_right = 0;

    for (uint16_t i = 0; i < sample_count; ++i) {
        sum_front_left += readAnalogOversampled(PIN_IR_FRONT_LEFT);
        sum_rear_left += readAnalogOversampled(PIN_IR_REAR_LEFT);
        sum_front_right += readAnalogOversampled(PIN_IR_FRONT_RIGHT);
        sum_rear_right += readAnalogOversampled(PIN_IR_REAR_RIGHT);
        delay(5);
    }

    uint16_t avg_front_left = sum_front_left / sample_count;
    uint16_t avg_rear_left = sum_rear_left / sample_count;
    uint16_t avg_front_right = sum_front_right / sample_count;
    uint16_t avg_rear_right = sum_rear_right / sample_count;

    float left_dist = 0.5f * (voltageToDistanceMM(adcToVoltage(avg_front_left)) +
                              voltageToDistanceMM(adcToVoltage(avg_rear_left)));
    float right_dist = 0.5f * (voltageToDistanceMM(adcToVoltage(avg_front_right)) +
                               voltageToDistanceMM(adcToVoltage(avg_rear_right)));
    nominal_side_dist_mm_ = 0.5f * (left_dist + right_dist);

    Serial.printf("[IR] Calibrated side reference distance: %4.1fmm\n", nominal_side_dist_mm_);
    return true;
}

bool IRSensorArray::hasFrontWall() const {
    return readings_.wall_front;
}

bool IRSensorArray::hasLeftWall() const {
    return readings_.wall_left;
}

bool IRSensorArray::hasRightWall() const {
    return readings_.wall_right;
}

bool IRSensorArray::hasLeftPostRising() const {
    return post_rising_l_;
}

bool IRSensorArray::hasLeftPostFalling() const {
    return post_falling_l_;
}

bool IRSensorArray::hasRightPostRising() const {
    return post_rising_r_;
}

bool IRSensorArray::hasRightPostFalling() const {
    return post_falling_r_;
}

float IRSensorArray::getCenteringError() const {
    return readings_.centering_error;
}

DistanceSensors IRSensorArray::getReadings() const {
    return readings_;
}
