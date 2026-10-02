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
      nominal_side_dist_mm_(NOMINAL_SIDE_WALL_MM),
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
    Serial.println("[IR] Sharp GP2Y0A51SK0F (0A51SK) 5-Sensor Array Initialized (2-15 cm Range).");
}

float IRSensorArray::adcToVoltage(uint16_t raw_adc) {
    return ((float)raw_adc / ADC_RESOLUTION) * ADC_REF_VOLTAGE;
}

float IRSensorArray::voltageToDistanceMM(float voltage) {
    // Sharp GP2Y0A51SK0F model: D_mm = A / (V - B)
    if (voltage <= (SHARP_0A51SK_B + 0.05f)) {
        return SHARP_MAX_DIST_MM; // Beyond 150mm detection range
    }
    float dist = SHARP_0A51SK_A / (voltage - SHARP_0A51SK_B);
    return constrain(dist, SHARP_MIN_DIST_MM, SHARP_MAX_DIST_MM);
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
    uint16_t raw_f   = readAnalogOversampled(PIN_IR_FRONT);
    uint16_t raw_l45 = readAnalogOversampled(PIN_IR_LEFT_45);
    uint16_t raw_l90 = readAnalogOversampled(PIN_IR_LEFT_90);
    uint16_t raw_r45 = readAnalogOversampled(PIN_IR_RIGHT_45);
    uint16_t raw_r90 = readAnalogOversampled(PIN_IR_RIGHT_90);

    // 2. Exponential moving average low-pass filter
    const float alpha = 0.5f;
    readings_.front    = (uint16_t)(alpha * raw_f   + (1.0f - alpha) * readings_.front);
    readings_.left_45  = (uint16_t)(alpha * raw_l45 + (1.0f - alpha) * readings_.left_45);
    readings_.left_90  = (uint16_t)(alpha * raw_l90 + (1.0f - alpha) * readings_.left_90);
    readings_.right_45 = (uint16_t)(alpha * raw_r45 + (1.0f - alpha) * readings_.right_45);
    readings_.right_90 = (uint16_t)(alpha * raw_r90 + (1.0f - alpha) * readings_.right_90);

    // 3. Compute physical distances in millimeters via Sharp 0A51SK curve
    readings_.front_mm    = voltageToDistanceMM(adcToVoltage(readings_.front));
    readings_.left_45_mm  = voltageToDistanceMM(adcToVoltage(readings_.left_45));
    readings_.left_90_mm  = voltageToDistanceMM(adcToVoltage(readings_.left_90));
    readings_.right_45_mm = voltageToDistanceMM(adcToVoltage(readings_.right_45));
    readings_.right_90_mm = voltageToDistanceMM(adcToVoltage(readings_.right_90));

    // 4. Binary wall presence classification
    readings_.wall_front = (readings_.front > thresh_front_) || (readings_.front_mm < WALL_DETECT_DIST_MM);
    readings_.wall_left  = (readings_.left_90 > thresh_l90_) || (readings_.left_90_mm < WALL_DETECT_DIST_MM) || (readings_.left_45 > thresh_l45_);
    readings_.wall_right = (readings_.right_90 > thresh_r90_) || (readings_.right_90_mm < WALL_DETECT_DIST_MM) || (readings_.right_45 > thresh_r45_);

    // 5. Post / Pillar edge detection for cell longitudinal alignment
    int16_t delta_l90 = (int16_t)readings_.left_90 - (int16_t)prev_l90_;
    int16_t delta_r90 = (int16_t)readings_.right_90 - (int16_t)prev_r90_;

    post_rising_l_  = (delta_l90 > 70);
    post_falling_l_ = (delta_l90 < -70);
    post_rising_r_  = (delta_r90 > 70);
    post_falling_r_ = (delta_r90 < -70);

    prev_l90_ = readings_.left_90;
    prev_r90_ = readings_.right_90;

    // 6. Millimeter-Accurate Centering Error Calculation
    bool valid_left_guide  = (readings_.left_90_mm < 95.0f);
    bool valid_right_guide = (readings_.right_90_mm < 95.0f);

    if (valid_left_guide && valid_right_guide) {
        // Dual wall corridor centering in mm:
        // If left_90_mm < right_90_mm, robot is closer to left wall
        // err_mm = (left_dist - right_dist) / 2
        // e.g. Left = 40mm, Right = 60mm -> err_mm = -10mm (robot is 10mm to the left)
        float err_mm = (readings_.left_90_mm - readings_.right_90_mm) * 0.5f;
        // Normalize to [-1.0, 1.0] relative to a 20mm max corridor corridor deflection
        readings_.centering_error = err_mm / 25.0f;
    } else if (valid_left_guide) {
        // Single left wall guide:
        float err_mm = readings_.left_90_mm - nominal_side_dist_mm_;
        readings_.centering_error = err_mm / 25.0f;
    } else if (valid_right_guide) {
        // Single right wall guide:
        float err_mm = nominal_side_dist_mm_ - readings_.right_90_mm;
        readings_.centering_error = err_mm / 25.0f;
    } else {
        readings_.centering_error = 0.0f;
    }

    readings_.centering_error = constrain(readings_.centering_error, -1.0f, 1.0f);
}

bool IRSensorArray::calibrateInCell(uint16_t sample_count) {
    Serial.println("[IR] Calibrating Sharp GP2Y0A51SK0F Sensors in cell...");

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

    nominal_center_l90_ = avg_l90;
    nominal_center_r90_ = avg_r90;
    nominal_center_l45_ = avg_l45;
    nominal_center_r45_ = avg_r45;

    float l90_dist = voltageToDistanceMM(adcToVoltage(avg_l90));
    float r90_dist = voltageToDistanceMM(adcToVoltage(avg_r90));
    nominal_side_dist_mm_ = 0.5f * (l90_dist + r90_dist);

    thresh_l90_   = (uint16_t)(avg_l90 * 0.45f);
    thresh_r90_   = (uint16_t)(avg_r90 * 0.45f);
    thresh_l45_   = (uint16_t)(avg_l45 * 0.45f);
    thresh_r45_   = (uint16_t)(avg_r45 * 0.45f);
    thresh_front_ = (uint16_t)(avg_f * 0.45f);

    Serial.printf("[IR] Calibrated 0A51SK Baselines: L90=%4.1fmm, R90=%4.1fmm, NominalSide=%4.1fmm\n",
                  l90_dist, r90_dist, nominal_side_dist_mm_);
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
