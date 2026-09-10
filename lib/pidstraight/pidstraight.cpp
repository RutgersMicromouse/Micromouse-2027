#include "pidstraight.h"
#include "tof.h"

// Variable Settings
double max_speed = 200;

// PID for distance
double Kp_dist  = 6.0;
double Ki_dist  = 0;
double Kd_dist  = 0;


// PID for angle offset
double Kp_angle = 4.1;
double Ki_angle = 0;
double Kd_angle = 0.4;

// PID for lateral correction
double Kp_lat = 2.00;
double Kd_lat = 0.4;

double lat_error_old = 0;

// TOF
// double Kp_wall = 0.5;
// double Kd_wall = 0.0;


double identity_diag[8] = {0.0, 45, 90, 135, 180, 225, 270, 315};


// const double CELL_MM = 180.0;


// Opening detection window
// const double OPENING_START_MM = 55;
// const double OPENING_END_MM   = 120;


// Edge detection threshold
// const double OPENING_DELTA = 25;


void pidForward(double distance) {

    int goal_front_distance = -1;
    int possible_front_distance = front();
    int state = 0;

    // Snap goal angle to nearest 45°
    double goal_angle;
    int closest_index = 0;
    double arr_diag[8];
    for (int i = 0; i <= 7; i++) {
        arr_diag[i] = identity_diag[i] - angle();
        if (arr_diag[i] >  180) arr_diag[i] -= 360;
        if (arr_diag[i] < -180) arr_diag[i] += 360;
        if (abs(arr_diag[i]) < abs(arr_diag[closest_index])) closest_index = i;
    }
    goal_angle = identity_diag[closest_index];


    // Serial.print("  goal_angle:           "); Serial.println(goal_angle);
    // Serial.print("  current angle:        "); Serial.println(angle());


    // Angle PID state
    double error_angle      = goal_angle - angle();
    if (error_angle >  180) error_angle -= 360;
    if (error_angle < -180) error_angle += 360;


    double error_angle_old   = error_angle;
    double error_deriv_angle = 0;
    double angleOut          = 0;


    // Stall detection state
    double t_old     = micros();
    double lastErrorPrint = t_old;
    int    loopCount = 0;


    while (true) {
        // for(int i = 0; i < 3; i ++) {
        //     front();
        //     right();
        //     left();
        // }

        if (goal_front_distance < 0) {
            if(abs(possible_front_distance - front()) < 5) {
                if (state != 3) {
                    state++;
                } else {
                    Serial.print("Start: "); Serial.println(front());
                    

                    int unnormalized_goal_front_distance = front() - distance;

                    int distance_cells = (unnormalized_goal_front_distance + 90) / 180;

                    goal_front_distance = distance_cells * 180 + 50;

                    Serial.print("Stop: "); Serial.println(goal_front_distance);
                }
            } else {
                state = 0;
                possible_front_distance = front();
            }
        }

        double t_now = micros();
        double dt = (t_now - t_old) * 1e-6;
        if (dt < 0.001) dt = 0.001;
        loopCount++;

        double avg_enc    = front();
        double error_dist = avg_enc - goal_front_distance;

        // ── Hard encoder cutoff — catches overshoot ────────────────────────
        // if (avg_enc <= goal_front_distance ||
        //     abs(error_dist) < 5) {
        //     // delay(1000);
        //     // for (int i = 0; i < 2; i++) {
        //     //     digitalWrite(LED_BUILTIN, HIGH);  delay(200);
        //     //     digitalWrite(LED_BUILTIN, LOW); delay(200);
        //     // }
        //     // delay(1000);
        //     // Serial.println("STOP: encoder cutoff");
        //     // Serial.print("  avg_enc:    "); Serial.println(avg_enc);
        //     // Serial.print("  overshoot:  "); Serial.println(avg_enc - goal_distance);
        //     // Serial.print("  loops:      "); Serial.println(loopCount);
        //     setLeftPWM(-80); setRightPWM(-80);
        //     delay(25);
        //     setLeftPWM(0);   setRightPWM(0);
        //     Serial.println("Arrived");
        //     return;
        // }


        // ── Stop condition ─────────────────────────────────────────────────
        if (avg_enc <= goal_front_distance || abs(error_dist) < 5) {
            // Serial.println("STOP: target reached");
            // Serial.print("  avg_enc:    "); Serial.println(avg_enc);
            // Serial.print("  error_dist: "); Serial.println(error_dist);
            // Serial.print("  loops:      "); Serial.println(loopCount);
            setLeftPWM(-80); setRightPWM(-80);
            delay(25);
            setLeftPWM(0);   setRightPWM(0);
            Serial.print("Arrived"); Serial.println(front());
            return;
        }


        // ── Guard: wall too close ──────────────────────────────────────────
        if (front() > 0 && front() < 50 && t_now > t_old + 100000UL) { // only checks after .1 seconds
            Serial.println("STOP: front wall guard triggered");
            Serial.print("  front():    "); Serial.println(front());
            Serial.print("  avg_enc:    "); Serial.println(avg_enc);
            Serial.print("  error_dist: "); Serial.println(error_dist);
            digitalWrite(LED_BUILTIN, HIGH);  delay(25);
            digitalWrite(LED_BUILTIN, LOW); delay(25);
            setLeftPWM(0); setRightPWM(0);
            return;
        }


        // ── Guard: stall detection ─────────────────────────────────────────
        /*if (t_now > t_old + 5000000UL) {
            Serial.println(t_now);
            Serial.println(t_old + 5000000UL);
            double dR = abs(encRight.read() - sampleRight);
            double dL = abs(encLeft.read()  - sampleLeft);
            Serial.print("  [stall check] dL="); Serial.print(dL);
            Serial.print(" dR=");               Serial.println(dR);
            if (dL < 20 && dR < 20) {
                delay(1000);
                for (int i = 0; i < 6; i++) {
                    digitalWrite(LED_BUILTIN, HIGH);  delay(200);
                    digitalWrite(LED_BUILTIN, LOW); delay(200);
                }
                delay(1000);
                Serial.println("STOP: stall detected");
                setLeftPWM(0); setRightPWM(0);
                return;
            }
            sampleTime  = micros();
            sampleRight = encRight.read();
            sampleLeft  = encLeft.read();
        }*/


        // ── Distance PID / base speed ──────────────────────────────────────
        double distOut = Kp_dist * error_dist;
        double basePWM;
        if (error_dist > 400) {
            basePWM = max_speed;
        } else if (error_dist > 20) {
            basePWM = constrain(distOut, 40, max_speed);
        } else {
            basePWM = 0;
        }

        // ── Lateral correction ──────────────────────────────────────
        double lateral_error = 0;

        // only use wall if valid
        int leftDist = left();
        int rightDist = right();
        if (leftDist > 0 && rightDist > 0) {
            leftDist = leftDist % 180;
            rightDist = rightDist % 180;
            lateral_error =
                leftDist - (rightDist + leftDist) / 2;
        }

        double lat_deriv =
            (lateral_error - lat_error_old) / dt;

        if (!isfinite(lat_deriv))
            lat_deriv = 0;

        double lateralOut = Kp_lat * lateral_error + Kd_lat * lat_deriv;

        // steering correction in degrees
        lateralOut = constrain(lateralOut, -8, 8);

        lat_error_old = lateral_error;




        // ── Angle PID ──────────────────────────────────────

        double desired_angle = goal_angle + lateralOut;

        error_angle = desired_angle - angle();


        if (error_angle >  180) error_angle -= 360;
        if (error_angle < -180) error_angle += 360;


        error_deriv_angle = (error_angle - error_angle_old) / dt;


        if (!isfinite(error_deriv_angle)) {
            error_deriv_angle = 0;
        }


        angleOut =
            Kp_angle * error_angle +
            Kd_angle * error_deriv_angle;


        angleOut = constrain(angleOut, -60.0, 60.0);

        // ── Motor output ───────────────────────────────────
        double leftPWM  = basePWM - angleOut;
        double rightPWM = basePWM + angleOut;


        leftPWM  = constrain(leftPWM,  0.0, 255.0);
        rightPWM = constrain(rightPWM, 0.0, 255.0);


        setLeftPWM(leftPWM);
        setRightPWM(rightPWM);


        // ── Print abs(error_dist) every 50ms ──────────────────────────────
        if (micros() - lastErrorPrint > 100000) {
            // Serial.print("Speed: "); Serial.print(leftPWM); Serial.print(" | "); Serial.print(rightPWM);
            // Serial.print("  abs(error_dist): "); Serial.print(abs(error_dist));
            // Serial.print("  avg_enc: ");         Serial.println(avg_enc);
            
            // Serial.print("front=");
            // Serial.print(front());

            // Serial.print(" goal=");
            // Serial.print(goal_front_distance);

            // Serial.print(" errDist=");
            // Serial.print(error_dist);

            // Serial.print(" distOut=");
            // Serial.print(distOut);

            // Serial.print(" base=");
            // Serial.print(basePWM);

            // Serial.print(" latErr=");
            // Serial.print(lateral_error);

            // Serial.print(" latOut=");
            // Serial.print(lateralOut);

            // Serial.print(" angle=");
            // Serial.print(angle());

            // Serial.print(" goalAngle=");
            // Serial.print(goal_angle);

            // Serial.print(" errAngle=");
            // Serial.print(error_angle);

            // Serial.print(" angleOut=");
            // Serial.print(angleOut);

            // Serial.print(" L=");
            // Serial.print(leftPWM);

            // Serial.print(" R=");
            // Serial.println(rightPWM);

            lastErrorPrint = micros();
        }

        // ── Throttled full debug every 500 loops ───────────────────────────
        if (loopCount % 500 == 0) {
            Serial.println("--- loop ---");
            Serial.print("  loop#:       "); Serial.println(loopCount);
            Serial.print("  avg_enc:     "); Serial.println(avg_enc);
            Serial.print("  error_dist:  "); Serial.println(error_dist);
            Serial.print("  distOut:     "); Serial.println(distOut);
            Serial.print("  basePWM:     "); Serial.println(basePWM);
            Serial.print("  error_angle: "); Serial.println(error_angle);
            Serial.print("  angleOut:    "); Serial.println(angleOut);
            Serial.print("  leftPWM:     "); Serial.println(leftPWM);
            Serial.print("  rightPWM:    "); Serial.println(rightPWM);
            Serial.print("  front():     "); Serial.println(front());
        }


        error_angle_old = error_angle;
        t_old = t_now;
    }
}




void pidForwardLeftWallFollow() {
    Serial.println("=== pidForwardLeftWallFollow START ===");


    double goal_angle;
    int closest_index = 0;
    double arr_diag[8];


    for (int i = 0; i <= 7; i++) {
        arr_diag[i] = identity_diag[i] - angle();
        if (arr_diag[i] >  180) arr_diag[i] -= 360;
        if (arr_diag[i] < -180) arr_diag[i] += 360;
        if (abs(arr_diag[i]) < abs(arr_diag[closest_index])) closest_index = i;
    }
    goal_angle = identity_diag[closest_index];


    Serial.print("  goal_angle:    "); Serial.println(goal_angle);
    Serial.print("  current angle: "); Serial.println(angle());


    double t_old             = micros();
    double error_angle       = goal_angle - angle();
    if (error_angle >  180) error_angle -= 360;
    if (error_angle < -180) error_angle += 360;


    double error_int_angle   = 0;
    double error_deriv_angle = 0;
    double error_angle_old   = error_angle;
    double angleOut          = 0;


    double sampleTime  = micros();
    double sampleRight = encRight.read();
    double sampleLeft  = encLeft.read();
    int    loopCount   = 0;
    double lastErrorPrint = micros();


    while (true) {
        loopCount++;


        // ── Print every 50ms ───────────────────────────────────────────────
        if (micros() - lastErrorPrint > 50000) {
            Serial.print("  [wallfollow] front(): "); Serial.print(front());
            Serial.print("  leftWall(): ");          Serial.print(leftWall());
            Serial.print("  angle(): ");             Serial.println(angle());
            lastErrorPrint = micros();
        }


        if (!leftWall()) {
            Serial.println("STOP: left wall gone");
            Serial.print("  loops="); Serial.println(loopCount);
            delay(10);
            setLeftPWM(0); setRightPWM(0);
            return;
        }


        if (micros() > sampleTime + 100000UL) {
            double dR = abs(encRight.read() - sampleRight);
            double dL = abs(encLeft.read()  - sampleLeft);
            Serial.print("  [stall check] dL="); Serial.print(dL);
            Serial.print(" dR=");               Serial.println(dR);
            if (dL < 2 || dR < 2) {
                Serial.println("STOP: stall detected");
                setLeftPWM(0); setRightPWM(0);
                return;
            }
            sampleTime  = micros();
            sampleRight = encRight.read();
            sampleLeft  = encLeft.read();
        }


        if (front() > 0 && front() < 60) {
            Serial.println("STOP: front wall");
            Serial.print("  front()="); Serial.println(front());
            setLeftPWM(0); setRightPWM(0);
            return;
        }


        double t_now = micros();
        double dt    = t_now - t_old;
        if (dt < 1) dt = 1;


        error_angle = goal_angle - angle();
        if (error_angle >  180) error_angle -= 360;
        if (error_angle < -180) error_angle += 360;


        error_int_angle  += error_angle * dt;
        error_deriv_angle = (error_angle - error_angle_old) / dt;


        angleOut = Kp_angle * error_angle
                 + Ki_angle * error_int_angle
                 + Kd_angle * error_deriv_angle;
        angleOut = constrain(angleOut, -60.0, 60.0);


        setLeftPWM (200 - angleOut);
        setRightPWM(200 + angleOut);


        error_angle_old = error_angle;
        t_old = t_now;
    }
}

void pidReverse(double distance) {
    double goal_distance =
       (TICKS_PER_ROTATION * distance) / (WHEEL_DIAM * PI);


    encLeft.write(0);
    encRight.write(0);


    double goal_angle = angle();


    while (true) {


        double leftEnc  = abs(encLeft.read());
        double rightEnc = abs(encRight.read());


        double avg_enc = (leftEnc + rightEnc) / 2.0;


        if (avg_enc >= goal_distance) {
            setLeftPWM(0);
            setRightPWM(0);
            return;
        }


        double error_angle = goal_angle - angle();


        if (error_angle > 180) error_angle -= 360;
        if (error_angle < -180) error_angle += 360;


       double angleOut = Kp_angle * error_angle;


        double leftPWM  = -140 - angleOut;
        double rightPWM = -140 + angleOut;


        setLeftPWM(leftPWM);
        setRightPWM(rightPWM);
    }
}





