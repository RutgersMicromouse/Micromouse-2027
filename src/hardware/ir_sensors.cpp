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
    readings_.wall_left = readings_.front_left >= thresh_side_ && readings_.rear_left >= thresh_side_;
    readings_.wall_right = readings_.front_right >= thresh_side_ && readings_.rear_right >= thresh_side_;

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
    bool valid_front_left = readings_.front_left >= thresh_side_ && readings_.front_left_mm < WALL_DETECT_DIST_MM;
    bool valid_rear_left = readings_.rear_left >= thresh_side_ && readings_.rear_left_mm < WALL_DETECT_DIST_MM;
    bool valid_front_right = readings_.front_right >= thresh_side_ && readings_.front_right_mm < WALL_DETECT_DIST_MM;
    bool valid_rear_right = readings_.rear_right >= thresh_side_ && readings_.rear_right_mm < WALL_DETECT_DIST_MM;
    bool valid_left_guide = valid_front_left || valid_rear_left;
    bool valid_right_guide = valid_front_right || valid_rear_right;

    float left_dist = valid_front_left && valid_rear_left
        ? 0.5f * (readings_.front_left_mm + readings_.rear_left_mm)
        : valid_front_left ? readings_.front_left_mm : readings_.rear_left_mm;
    float right_dist = valid_front_right && valid_rear_right
        ? 0.5f * (readings_.front_right_mm + readings_.rear_right_mm)
        : valid_front_right ? readings_.front_right_mm : readings_.rear_right_mm;

    if (valid_left_guide && valid_right_guide) {
        // Positive means the mouse is closer to the left wall and must steer
        // right; negative means it is closer to the right wall and must steer
        // left.  This convention is consumed by MotionController.
        readings_.centering_error = (right_dist - left_dist) / 50.0f;
    } else if (valid_left_guide) {
        readings_.centering_error = (nominal_side_dist_mm_ - left_dist) / 25.0f;
    } else if (valid_right_guide) {
        readings_.centering_error = (right_dist - nominal_side_dist_mm_) / 25.0f;
    } else {
        readings_.centering_error = 0.0f;
    }

    readings_.centering_error = constrain(readings_.centering_error, -1.0f, 1.0f);

    // A yawed robot sees different distances at the front and rear sensors.
    float alignment_error = 0.0f;
    uint8_t alignment_sides = 0;
    if (valid_front_left && valid_rear_left) {
        alignment_error += atan2f(readings_.front_left_mm - readings_.rear_left_mm,
                                  SIDE_SENSOR_SPACING_MM) * (180.0f / 3.1415926535f);
        ++alignment_sides;
    }
    if (valid_front_right && valid_rear_right) {
        alignment_error -= atan2f(readings_.front_right_mm - readings_.rear_right_mm,
                                  SIDE_SENSOR_SPACING_MM) * (180.0f / 3.1415926535f);
        ++alignment_sides;
    }
    readings_.wall_alignment_error_deg = alignment_sides > 0
        ? constrain(alignment_error / alignment_sides, -15.0f, 15.0f)
        : 0.0f;
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
