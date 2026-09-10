#include "pidrotate.h"

double Kp = 8.0;
double Ki = 0.5;
double Kd = 1.0;

void pidRotate(double turn_angle) {
    turnTo(angle() + turn_angle);
}

void turnTo(double goal_angle) {
    double error = goal_angle - angle();
    // Serial.print("Initial Angle:");
    // Serial.println(angle());
    // Serial.print("Goal Angle:");
    // Serial.println(goal_angle);
    if (error > 180)  error -= 360;
    if (error < -180) error += 360;

    double error_old = error;
    double error_int = 0;
    double t_old = micros();
    double startTime = micros();

    while (true) {
        // 1. Reached goal
        if (abs(error) <= 1.0) {
            setRightPWM(0); setLeftPWM(0);
            // Serial.println("turnTo: done");
            return;
        }

        // 2. Timeout only — no encoder stall detection
        if (micros() - startTime > 5e6) {
            setRightPWM(0); setLeftPWM(0);
            Serial.println("turnTo: stall");
            return;
        }

        double currentAngle = angle();
        error = goal_angle - currentAngle;
        if (error < -180.0) error += 360;
        else if (error > 180) error -= 360;

        double now = micros();
        double dt  = (now - t_old) / 1e6;
        if (dt <= 0) { t_old = now; continue; }

        error_int += error * dt;

        // Clamp integral to prevent massive windup
        error_int = constrain(error_int, -50, 50);

        double error_deriv = (error - error_old) / dt;

        double angleOut = Kp * error + Ki * error_int + Kd * error_deriv;

        if (abs(error) > 1.0) {
            if (angleOut > 0 && angleOut < 100)  angleOut = 100;
            if (angleOut < 0 && angleOut > -100) angleOut = -100;
        }
        angleOut = constrain(angleOut, -255, 255);

        setLeftPWM(-angleOut);
        setRightPWM(angleOut);

        error_old = error;
        t_old = now;
    }
}
