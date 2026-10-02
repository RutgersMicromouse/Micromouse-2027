#include "ir_sensors.h"

IRSensorArray ir_sensors;

IRSensorArray::IRSensorArray()
    : thresh_front_(IR_WALL_DETECT_FRONT),
      thresh_l45_(IR_WALL_DETECT_L45),
      thresh_l90_(IR_WALL_DETECT_L90),
      thresh_r45_(IR_WALL_DETECT_R45),
      thresh_r90_(IR_WALL_DETECT_R90),
      nominal_center_l90_(IR_NOMINAL_CENTER_L90),
      nominal_center_r90_(IR_NOMINAL_CENTER_R90),
      nominal_center_l45_(IR_NOMINAL_CENTER_L45),
      nominal_center_r45_(IR_NOMINAL_CENTER_R45),
      prev_l90_(0),
      prev_r90_(0),
      post_rising_l_(false),
      post_falling_l_(false),
      post_rising_r_(false),
      post_falling_r_(false) {
    memset(&readings_, 0, sizeof(readings_));
}

void IRSensorArray::begin() {
    pinMode(PIN_IR_FRONT, INPUT);
    pinMode(PIN_IR_LEFT_45, INPUT);
    pinMode(PIN_IR_LEFT_90, INPUT);
    pinMode(PIN_IR_RIGHT_45, INPUT);
    pinMode(PIN_IR_RIGHT_90, INPUT);

    // Initial warm-up read
    update();
    Serial.println("[IR] 5-Channel Analog IR Distance Sensors initialized (FIR:17, L1:16, L2:15, R1:14, R2:20).");
}

uint16_t IRSensorArray::readAnalogOversampled(uint8_t pin, uint8_t samples) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < samples; ++i) {
        sum += analogRead(pin);
    }
    return (uint16_t)(sum / samples);
}

void IRSensorArray::update() {
    // Read raw analog values with oversampling
    uint16_t raw_f   = readAnalogOversampled(PIN_IR_FRONT);
    uint16_t raw_l45 = readAnalogOversampled(PIN_IR_LEFT_45);
    uint16_t raw_l90 = readAnalogOversampled(PIN_IR_LEFT_90);
    uint16_t raw_r45 = readAnalogOversampled(PIN_IR_RIGHT_45);
    uint16_t raw_r90 = readAnalogOversampled(PIN_IR_RIGHT_90);

    // Low-pass exponential moving average filter (alpha = 0.5)
    const float alpha = 0.5f;
    readings_.front    = (uint16_t)(alpha * raw_f   + (1.0f - alpha) * readings_.front);
    readings_.left_45  = (uint16_t)(alpha * raw_l45 + (1.0f - alpha) * readings_.left_45);
    readings_.left_90  = (uint16_t)(alpha * raw_l90 + (1.0f - alpha) * readings_.left_90);
    readings_.right_45 = (uint16_t)(alpha * raw_r45 + (1.0f - alpha) * readings_.right_45);
    readings_.right_90 = (uint16_t)(alpha * raw_r90 + (1.0f - alpha) * readings_.right_90);

    // Binary wall presence logic
    readings_.wall_front = (readings_.front > thresh_front_);
    readings_.wall_left  = (readings_.left_90 > thresh_l90_) || (readings_.left_45 > thresh_l45_);
    readings_.wall_right = (readings_.right_90 > thresh_r90_) || (readings_.right_45 > thresh_r45_);

    // Post / Pillar edge detection for cell longitudinal alignment
    int16_t delta_l90 = (int16_t)readings_.left_90 - (int16_t)prev_l90_;
    int16_t delta_r90 = (int16_t)readings_.right_90 - (int16_t)prev_r90_;

    post_rising_l_  = (delta_l90 > 80);
    post_falling_l_ = (delta_l90 < -80);
    post_rising_r_  = (delta_r90 > 80);
    post_falling_r_ = (delta_r90 < -80);

    prev_l90_ = readings_.left_90;
    prev_r90_ = readings_.right_90;

    // Centering error computation
    bool valid_left_guide  = (readings_.left_90 > thresh_l90_) && (readings_.left_45 > thresh_l45_ * 0.8f);
    bool valid_right_guide = (readings_.right_90 > thresh_r90_) && (readings_.right_45 > thresh_r45_ * 0.8f);

    if (valid_left_guide && valid_right_guide) {
        // Dual wall corridor centering:
        // Difference between left error and right error
        float err_l = (float)readings_.left_90 - (float)nominal_center_l90_;
        float err_r = (float)readings_.right_90 - (float)nominal_center_r90_;
        readings_.centering_error = (err_l - err_r) / (float)nominal_center_l90_;
    } else if (valid_left_guide) {
        // Single left wall guide (right wall opened):
        float err_l = (float)readings_.left_90 - (float)nominal_center_l90_;
        readings_.centering_error = 2.0f * (err_l / (float)nominal_center_l90_);
    } else if (valid_right_guide) {
        // Single right wall guide (left wall opened):
        float err_r = (float)readings_.right_90 - (float)nominal_center_r90_;
        readings_.centering_error = -2.0f * (err_r / (float)nominal_center_r90_);
    } else {
        // Open intersection / no walls: zero centering error, rely on IMU heading hold
        readings_.centering_error = 0.0f;
    }

    // Clamp centering error to [-1.0, 1.0]
    readings_.centering_error = constrain(readings_.centering_error, -1.0f, 1.0f);
}

bool IRSensorArray::calibrateInCell(uint16_t sample_count) {
    Serial.println("[IR] Starting In-Cell Auto-Calibration...");
    Serial.println("[IR] Make sure the robot is placed in the center of a cell with Left, Right, and Front walls!");

    uint32_t sum_f = 0, sum_l45 = 0, sum_l90 = 0, sum_r45 = 0, sum_r90 = 0;

    for (uint16_t i = 0; i < sample_count; ++i) {
        sum_f   += readAnalogOversampled(PIN_IR_FRONT);
        sum_l45 += readAnalogOversampled(PIN_IR_LEFT_45);
        sum_l90 += readAnalogOversampled(PIN_IR_LEFT_90);
        sum_r45 += readAnalogOversampled(PIN_IR_RIGHT_45);
        sum_r90 += readAnalogOversampled(PIN_IR_RIGHT_90);
        delay(5);
    }

    uint16_t avg_f   = sum_f   / sample_count;
    uint16_t avg_l45 = sum_l45 / sample_count;
    uint16_t avg_l90 = sum_l90 / sample_count;
    uint16_t avg_r45 = sum_r45 / sample_count;
    uint16_t avg_r90 = sum_r90 / sample_count;

    Serial.printf("[IR] Measured Cell Baselines: F=%d, L45=%d, L90=%d, R45=%d, R90=%d\n",
                  avg_f, avg_l45, avg_l90, avg_r45, avg_r90);

    // Save nominal centers
    nominal_center_l90_ = avg_l90;
    nominal_center_r90_ = avg_r90;
    nominal_center_l45_ = avg_l45;
    nominal_center_r45_ = avg_r45;

    // Set detection thresholds to 40% of nominal wall distance
    thresh_l90_   = (uint16_t)(avg_l90 * 0.40f);
    thresh_r90_   = (uint16_t)(avg_r90 * 0.40f);
    thresh_l45_   = (uint16_t)(avg_l45 * 0.40f);
    thresh_r45_   = (uint16_t)(avg_r45 * 0.40f);
    thresh_front_ = (uint16_t)(avg_f * 0.40f);

    Serial.println("[IR] In-Cell Calibration Complete!\n");
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
