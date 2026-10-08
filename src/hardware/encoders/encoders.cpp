#include "hardware/encoders/encoders.h"

// ==============================================================================
// ENCODERS
// ==============================================================================

Encoders::Encoders()
    : speed_filter_alpha_(ENCODER_SPEED_FILTER_ALPHA),
      distance_scale_(1.0f),
      invert_left_(INVERT_LEFT_ENCODER),
      invert_right_(INVERT_RIGHT_ENCODER),
      left_dist_acc_mm_(0.0f),
      right_dist_acc_mm_(0.0f) {
    memset(&state_, 0, sizeof(state_));
}

void Encoders::begin() {
    // Unit 0 for Left motor encoder
    initPcntUnit(PCNT_UNIT_0, PIN_ENC_L_A, PIN_ENC_L_B);

    // Unit 1 for Right motor encoder
    initPcntUnit(PCNT_UNIT_1, PIN_ENC_R_A, PIN_ENC_R_B);

    reset();
}

void Encoders::initPcntUnit(pcnt_unit_t unit, int pin_a, int pin_b) {
    // Enable internal pullups for N20 magnetic Hall effect encoder sensors
    pinMode(pin_a, INPUT_PULLUP);
    pinMode(pin_b, INPUT_PULLUP);

    // Channel 0: Pulse on Pin A, Level on Pin B
    pcnt_config_t config_ch0 = {
        .pulse_gpio_num = pin_a,
        .ctrl_gpio_num = pin_b,
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_REVERSE,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DEC,
        .counter_h_lim = 32767,
        .counter_l_lim = -32768,
        .unit = unit,
        .channel = PCNT_CHANNEL_0,
    };
    pcnt_unit_config(&config_ch0);

    // Channel 1: Pulse on Pin B, Level on Pin A (Completes 4x quadrature decoding)
    pcnt_config_t config_ch1 = {
        .pulse_gpio_num = pin_b,
        .ctrl_gpio_num = pin_a,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DEC,
        .counter_h_lim = 32767,
        .counter_l_lim = -32768,
        .unit = unit,
        .channel = PCNT_CHANNEL_1,
    };
    pcnt_unit_config(&config_ch1);

    // Glitch filter: ignore pulses shorter than ~100 APB clock cycles (~1.25 µs)
    pcnt_set_filter_value(unit, 100);
    pcnt_filter_enable(unit);

    // Clear and resume counting
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);
}

void Encoders::update(float dt_seconds) {
    if (dt_seconds <= 0.0f) {
        dt_seconds = CONTROL_DT_S;
    }

    int16_t raw_left = 0;
    int16_t raw_right = 0;

    // Read counter hardware registers and immediately clear for the next delta
    pcnt_get_counter_value(PCNT_UNIT_0, &raw_left);
    pcnt_counter_clear(PCNT_UNIT_0);

    pcnt_get_counter_value(PCNT_UNIT_1, &raw_right);
    pcnt_counter_clear(PCNT_UNIT_1);

    if (invert_left_)  raw_left  = -raw_left;
    if (invert_right_) raw_right = -raw_right;

    state_.left_delta_ticks = raw_left;
    state_.right_delta_ticks = raw_right;

    state_.left_ticks_total += raw_left;
    state_.right_ticks_total += raw_right;

    // Convert ticks to millimeters
    // Dividing by the distance scale makes the robot count less per tick, so it drives further
    float d_left_mm = (float)raw_left * MM_PER_TICK_LEFT / distance_scale_;
    float d_right_mm = (float)raw_right * MM_PER_TICK_RIGHT / distance_scale_;

    left_dist_acc_mm_ += d_left_mm;
    right_dist_acc_mm_ += d_right_mm;

    state_.left_dist_mm = left_dist_acc_mm_;
    state_.right_dist_mm = right_dist_acc_mm_;

    // Instantaneous wheel speeds (mm/s)
    float raw_speed_l = d_left_mm / dt_seconds;
    float raw_speed_r = d_right_mm / dt_seconds;

    // First-order low-pass filter on the speeds. One tick is a large step in speed at low speed
    // (see ENCODER_SPEED_FILTER_ALPHA), and unfiltered that step goes straight to the motors.
    const float alpha = speed_filter_alpha_;
    state_.left_speed_mm_s  = (alpha * raw_speed_l) + ((1.0f - alpha) * state_.left_speed_mm_s);
    state_.right_speed_mm_s = (alpha * raw_speed_r) + ((1.0f - alpha) * state_.right_speed_mm_s);

    state_.linear_speed_mm_s = (state_.left_speed_mm_s + state_.right_speed_mm_s) * 0.5f;
}

EncoderState Encoders::getState() const {
    return state_;
}

void Encoders::reset() {
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_1);
    left_dist_acc_mm_ = 0.0f;
    right_dist_acc_mm_ = 0.0f;
    memset(&state_, 0, sizeof(state_));
}

void Encoders::setInverted(bool invert_left, bool invert_right) {
    invert_left_ = invert_left;
    invert_right_ = invert_right;
}

void Encoders::getInverted(bool& invert_left, bool& invert_right) const {
    invert_left = invert_left_;
    invert_right = invert_right_;
}
