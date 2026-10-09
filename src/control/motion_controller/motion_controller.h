#pragma once

// Runs one MotionCommand at a time: turns "move one cell" / "turn left 90" into motor effort

#include "config.h"
#include "types.h"
#include "hardware/encoders/encoders.h"
#include "hardware/motors/motors.h"
#include "hardware/ir_sensors/ir_sensors.h"
#include "hardware/imu/imu.h"
#include "control/pid/pid.h"
#include "control/profile/profile.h"

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
    void setSyncGain(float k_sync) { setTune(TUNE_K_SYNC, k_sync); }
    float getSyncGain() const { return tune_[TUNE_K_SYNC]; }

    // Velocity PID Gain Tuning
    void setLinearVelGains(float kp, float ki, float kd) { setTune(TUNE_V_KP, kp); setTune(TUNE_V_KI, ki); setTune(TUNE_V_KD, kd); }
    void getLinearVelGains(float& kp, float& ki, float& kd) const { pid_linear_vel_.getGains(kp, ki, kd); }

    // Flash NVS persistence for controller tuning
    // Live tuning: the gains that the phone app's Tuning card and the `tune` console command can
    // change while the robot is switched on. The list itself is kTune in motion_controller.cpp,
    // in this order. NEW ENTRIES GO AT THE END: the phone app and the saved set go by position
    // and name, and main.cpp prints a few of these by number.
    enum TuneIndex {
        TUNE_V_KP, TUNE_V_KI, TUNE_V_KD,       // Speed loop (wheel speed -> motor effort)
        TUNE_H_KP, TUNE_H_KI, TUNE_H_KD,       // Heading loop (heading error -> turning effort)
        TUNE_K_SYNC, TUNE_ENC_A, TUNE_IMU_A, TUNE_DIST_K,
        TUNE_D_KP, TUNE_D_KI, TUNE_D_KD,       // Distance loop (distance error -> speed)
        TUNE_W_KP, TUNE_W_KI, TUNE_W_KD,       // Wall centring (IR error -> degrees of steering)
        TUNE_FF_KS, TUNE_FF_KV, TUNE_FF_KA,    // Feedforward: effort to hold a speed without any loop
        TUNE_TURN_FF,                          // Scale on the feedforward of turns and curves
        TUNE_H_MAX, TUNE_H_IMAX,               // Heading loop: most effort it may ask for, and of that from I
        TUNE_W_MAX,                            // Wall centring: most steering it may add, degrees
        TUNE_W_GYRO,                           // Wall centring: damping from the turn rate
        TUNE_S_DAMP,                           // Straights and diagonals: damping from the turn rate
        TUNE_T_DAMP, TUNE_T_PUSH,              // Settling after a turn on the spot: damping, and the nudge
        TUNE_C_KP,                             // Smooth curves: heading P added to h_kp while curving
        TUNE_COUNT
    };
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
    float tune_[TUNE_COUNT];       // The live tuning values (see TuneIndex); applyTuning() puts them to work
    void applyTuning();

    // Effort a wheel needs to hold `speed` while accelerating at `accel`:
    // ff_ks (friction) + ff_kv * speed + ff_ka * acceleration, from the tuning table
    float wheelFeedforward(float speed_mm_s, float accel_mm_s2) const;
};
