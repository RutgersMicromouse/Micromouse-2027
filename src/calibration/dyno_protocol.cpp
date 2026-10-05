#include "dyno_protocol.h"

#if ENABLE_DYNO_CALIBRATION

namespace DynoProtocol {

bool handleCommand(const String& cmd, Motors& motors, MotionController& motion,
                   QueueHandle_t motion_queue, std::function<void(const char*)> reply) {
    if (!cmd.startsWith("dyno")) return false;

    if (cmd == "dyno ping") {
        reply("DYNO_ACK READY");
        return true;
    }

    if (cmd.startsWith("dyno step ")) {
        float dl = 0.0f, dr = 0.0f;
        int dur_ms = 1000;
        if (sscanf(cmd.c_str() + 10, "%f %f %d", &dl, &dr, &dur_ms) >= 2) {
            motion.setCalibrating(true);
            motors.setRawEffort(dl, dr);
            delay(dur_ms);
            motors.brake();
            motion.setCalibrating(false);
            reply("DYNO_ACK STEP_DONE");
        } else {
            reply("ERR: USAGE 'dyno step <dl> <dr> <ms>'");
        }
        return true;
    }

    if (cmd.startsWith("dyno trap ")) {
        float spd = 300.0f, acc = 2000.0f, dist = 500.0f;
        if (sscanf(cmd.c_str() + 10, "%f %f %f", &spd, &acc, &dist) >= 3) {
            MotionCommand mc;
            mc.action = ACTION_MOVE_DISTANCE;
            mc.param_value = dist;
            mc.max_speed_mm_s = spd;
            mc.acceleration = acc;
            mc.enable_wall_centering = false;
            mc.entry_speed_mm_s = 0.0f;
            mc.exit_speed_mm_s = 0.0f;
            xQueueSend(motion_queue, &mc, 0);
            reply("DYNO_ACK TRAP_STARTED");
        } else {
            reply("ERR: USAGE 'dyno trap <spd> <acc> <dist>'");
        }
        return true;
    }

    if (cmd.startsWith("dyno trim ")) {
        float tl = 1.0f, tr = 1.0f;
        if (sscanf(cmd.c_str() + 10, "%f %f", &tl, &tr) == 2) {
            motors.setTrim(tl, tr);
            reply("DYNO_ACK TRIM_SET");
        }
        return true;
    }

    if (cmd.startsWith("dyno deadband ")) {
        float dl = 0.02f, dr = 0.02f;
        if (sscanf(cmd.c_str() + 14, "%f %f", &dl, &dr) == 2) {
            motors.setDeadband(dl, dr);
            reply("DYNO_ACK DEADBAND_SET");
        }
        return true;
    }

    if (cmd.startsWith("dyno sync ")) {
        float ks = 0.0004f;
        if (sscanf(cmd.c_str() + 10, "%f", &ks) == 1) {
            motion.setSyncGain(ks);
            reply("DYNO_ACK SYNC_SET");
        }
        return true;
    }

    if (cmd.startsWith("dyno pid ")) {
        float kp = 0.0025f, ki = 0.0005f, kd = 0.00005f;
        if (sscanf(cmd.c_str() + 9, "%f %f %f", &kp, &ki, &kd) == 3) {
            motion.setLinearVelGains(kp, ki, kd);
            reply("DYNO_ACK PID_SET");
        }
        return true;
    }

    if (cmd == "dyno save") {
        motors.saveToNVS();
        motion.saveToNVS();
        reply("DYNO_ACK SAVED_TO_NVS");
        return true;
    }

    if (cmd == "dyno get") {
        float tl = 1.0f, tr = 1.0f, dl = 0.02f, dr = 0.02f;
        motors.getTrim(tl, tr);
        motors.getDeadband(dl, dr);
        float ks = motion.getSyncGain();
        float kp = 0, ki = 0, kd = 0;
        motion.getLinearVelGains(kp, ki, kd);
        char buf[128];
        snprintf(buf, sizeof(buf), "DYNO_PARAMS: Trim=[%.3f,%.3f] Dead=[%.3f,%.3f] Sync=%.6f PID=[%.5f,%.5f,%.6f]",
                 tl, tr, dl, dr, ks, kp, ki, kd);
        reply(buf);
        return true;
    }

    return false;
}

} // namespace DynoProtocol

#endif // ENABLE_DYNO_CALIBRATION
