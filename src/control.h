#pragma once

// Turns "move one cell" / "turn left 90" into motor effort, 500 times a second:
//   PIDController      - generic PID loop
//   TrapezoidalProfile - accelerate / cruise / decelerate target generator
//   MotionController   - runs one MotionCommand at a time using the two above

#include "config.h"
#include "types.h"
#include "hardware.h"

// ==============================================================================
// PID CONTROLLER
// ==============================================================================

#include <Arduino.h>

class PIDController {
public:
    PIDController();
    PIDController(float kp, float ki, float kd, float max_out, float max_integral = 0.0f);

    void setGains(float kp, float ki, float kd);
    void getGains(float& kp, float& ki, float& kd) const { kp = kp_; ki = ki_; kd = kd_; }
    void setOutputLimits(float max_out, float max_integral = 0.0f);
    void reset();

    // Compute PID output given current error and time delta
    float update(float error, float dt_seconds);

private:
    float kp_;
    float ki_;
    float kd_;
    float max_output_;
    float max_integral_;

    float integral_;
    float prev_error_;
    float prev_derivative_;
    bool first_run_;
};

// ==============================================================================
// TRAPEZOIDAL PROFILE
// ==============================================================================

#include <Arduino.h>

class TrapezoidalProfile {
public:
    TrapezoidalProfile();

    // Start a new trapezoidal movement trajectory with optional initial and final boundary speeds
    void start(float target_distance, float max_speed, float acceleration,
               float start_speed = 0.0f, float end_speed = 0.0f);

    // Compute target position and velocity for the current time step
    void update(float dt_seconds);

    float getTargetDistance() const;
    float getTargetVelocity() const;
    bool isFinished() const;
    void stop();

private:
    float target_total_dist_;
    float max_speed_;
    float accel_;
    float v_start_;
    float v_peak_;
    float v_end_;

    float current_dist_;
    float current_vel_;

    float d_accel_;
    float d_cruise_;
    float d_decel_;

    float t_accel_;
    float t_cruise_;
    float t_decel_;
    float t_total_;
    float elapsed_time_;

    bool finished_;
};

// ==============================================================================
// MOTION CONTROLLER
// ==============================================================================

#include <Arduino.h>

// One snapshot of planned vs. actual motion (24 bytes). Read back with the console command "log".
struct RunLogSample {
    int16_t target_speed_mm_s;
    int16_t speed_mm_s;
    int16_t heading_x10;        // Heading in tenths of a degree, wrapped to +/-180°
    int16_t heading_err_x10;    // Wanted heading minus actual, tenths of a degree
    int8_t  forward_pct;        // Forward effort, percent
    int8_t  turn_pct;           // Turning effort, percent
    uint8_t action;             // MotionAction being executed
    uint8_t reserved;
    uint16_t ir[6];             // L90, L45, FL, FR, R45, R90
};

// Ring buffer holding the most recent RUN_LOG_SAMPLES samples
class RunLog {
public:
    RunLog() : head_(0), count_(0) {}

    void record(const RunLogSample& sample) {
        samples_[head_] = sample;
        head_ = (head_ + 1) % RUN_LOG_SAMPLES;
        if (count_ < RUN_LOG_SAMPLES) count_++;
    }

    void clear() { head_ = 0; count_ = 0; }
    uint16_t size() const { return count_; }

    // index 0 = oldest sample still held
    const RunLogSample& at(uint16_t index) const {
        return samples_[(head_ + RUN_LOG_SAMPLES - count_ + index) % RUN_LOG_SAMPLES];
    }

private:
    RunLogSample samples_[RUN_LOG_SAMPLES];
    uint16_t head_;
    uint16_t count_;
};

class MotionController {
public:
    MotionController(Encoders& encoders, Motors& motors, IRSensors& ir, IMU& imu);
    void begin();

    // Call strictly at 1kHz inside Core 1 task
    void update(float dt_seconds);

    // Command interface
    void executeCommand(const MotionCommand& cmd);
    bool isCommandFinished() const;
    void emergencyStop();

    // Reset reference frames
    void resetTracking();

    // Zero only the heading reference (current facing becomes 0°); distance tracking is untouched
    void resetHeading();

    // True once after a stall / encoder-fault / lost-heading emergency stop, so the navigator can abort the run
    // instead of sending the next move
    bool consumeSafetyStop() {
        bool tripped = safety_stop_;
        safety_stop_ = false;
        return tripped;
    }

    // What the 45° sensors saw of the cell ahead during the move that just finished
    WallPreview getWallPreview() const;

    // Times a move that should have been followed at speed was not followed in time, so the
    // robot braked instead of rolling on (a run that should be continuous but looks stop-and-go)
    uint16_t getLateHandovers() const { return late_handovers_; }

    // How far the heading is from where the current (or last) move was meant to end, in degrees
    float getHeadingErrorDeg() const { return start_heading_deg_ + target_relative_angle_deg_ - accumulated_heading_deg_; }

    // True once per straight, at the moment its look-ahead window closes: that is when the 45°
    // sensors have made up their minds about the next cell's side walls. The navigation task
    // then prints the verdict (printPreviewReport) so it can be watched on the phone app.
    bool consumePreviewReady() {
        bool ready = preview_ready_;
        preview_ready_ = false;
        return ready;
    }
    void printPreviewReport() const;

    // Recording of the most recent driving (only written while a move is running)
    const RunLog& getRunLog() const { return run_log_; }
    void clearRunLog() { run_log_.clear(); }

    // Number of emergency stops caused by a dead encoder channel / jammed wheel since boot
    uint16_t getEncoderFaults() const { return encoder_faults_; }

    // Wall following enable/disable
    void setWallCenteringEnabled(bool enabled);

    // Pauses the controller so a bench routine (bench/calibration.cpp) can drive the motors directly
    bool isCalibrating() const { return calibrating_motors_; }
    void setCalibrating(bool cal) { calibrating_motors_ = cal; }

    // Differential Wheel Speed Synchronization Lock Gain
    void setSyncGain(float k_sync) { k_wheel_sync_ = k_sync; }
    float getSyncGain() const { return k_wheel_sync_; }

    // Velocity PID Gain Tuning
    void setLinearVelGains(float kp, float ki, float kd) { pid_linear_vel_.setGains(kp, ki, kd); }
    void getLinearVelGains(float& kp, float& ki, float& kd) const { pid_linear_vel_.getGains(kp, ki, kd); }

    // Flash NVS persistence for controller tuning
    // Live tuning: the gains that the phone app's Tuning card and the `tune` console command can
    // change while the robot is switched on. The list itself is kTune in control.cpp.
    static const int TUNE_COUNT = 10;
    static const char* tuneName(int index);        // Short name; also the key it is saved under
    static const char* tuneDescription(int index);
    static float tuneDefault(int index);           // The value in the code
    float getTune(int index) const;
    bool setTune(int index, float value);          // false = outside the allowed range, not changed
    void forgetSavedTuning();                      // Back to the values in the code, and wipe the saved set

    void saveToNVS();                              // Keeps the current tuning through power-off
    bool loadFromNVS();

private:
    Encoders& encoders_;
    Motors& motors_;
    IRSensors& ir_;
    IMU& imu_;

    // Control loops
    PIDController pid_linear_dist_;
    PIDController pid_linear_vel_;
    PIDController pid_angular_heading_;
    PIDController pid_wall_centering_;

    TrapezoidalProfile profile_linear_;
    TrapezoidalProfile profile_angular_;

    MotionCommand active_cmd_;
    bool command_active_;
    bool command_finished_;

    float start_distance_mm_;
    float start_heading_deg_;
    float target_relative_dist_mm_;
    float target_relative_angle_deg_;

    // Continuous unwrapped heading state
    float accumulated_heading_deg_;
    float prev_raw_heading_deg_;

    // Safety: stall detection counter
    uint16_t stall_count_;

    // Safety: one-wheel-dead detection
    uint16_t encoder_fault_ticks_;
    uint16_t heading_fault_ticks_;
    uint16_t encoder_faults_;

    // Alignment state
    uint16_t wall_align_timer_;
    uint16_t chained_coast_timer_;

    void snapHeadingToGrid();
    void checkPillarDriftCorrection(float current_dist_mm);
    void startCurve(const MotionCommand& cmd, float angle_deg, float default_length_mm);

    // Smooth-turn state (heading follows distance along the curve)
    bool curve_active_;
    float curve_length_mm_;

    // Chaining: where the previous move was meant to end, if it ended at speed
    bool chain_valid_;
    float chain_end_distance_mm_;

    // Search look-ahead: range of 45° readings seen in the sampling window of the current move
    void samplePreview(float dist_in_move_mm);
    uint16_t preview_min_l_, preview_max_l_, preview_min_r_, preview_max_r_;
    uint16_t preview_samples_;
    bool preview_reported_;        // This move's look-ahead verdict has already been announced
    volatile bool preview_ready_;  // Set by the control task, taken by the navigation task
    uint16_t front_min_, front_max_, front_samples_;
    bool front_uses_left_sensor_;

    // Diagonal centring: strongest recent reading of each row of posts (fraction of a centred wall)
    float diag_peak_left_;
    float diag_peak_right_;

    // Settling at the end of a move that stops, and how long the robot has been idle since
    uint16_t settle_ticks_;
    uint16_t settle_good_ticks_;       // Ticks in a row the robot has been on target and still
    uint16_t idle_ticks_;

    // Feedforward & logging
    float prev_target_speed_mm_s_;
    uint16_t log_tick_;
    RunLog run_log_;

    bool wall_centering_enabled_;
    volatile bool calibrating_motors_;
    volatile bool safety_stop_;
    volatile uint16_t late_handovers_;
    float k_wheel_sync_;
};
