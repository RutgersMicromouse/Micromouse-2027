#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>

const int POWER_ENABLE   = 13; // GPIO 13 ενεργοποιεί τη γραμμή τροφοδοσίας 12V
const int SDA_PIN        = 21; // SDA από το σχηματικό
const int SCL_PIN        = 20; // SCL από το σχηματικό
const int RIGHT_MOTOR_CH = 1;  // Δεξιός κινητήρας στο Κανάλι 1 (M1A/M1B)
const int LEFT_MOTOR_CH  = 2;  // Αριστερός κινητήρας στο Κανάλι 2 (M2A/M2B)

MotoronI2C mc;

void checkMotoronErrors() {
  uint16_t statusFlags = mc.getStatusFlags();
  if (statusFlags) {
    Serial.print("  [WARNING] Motoron Status Flags: 0x");
    Serial.println(statusFlags, HEX);
  }
}

void setup() {
  Serial.begin(115200);
  delay(3000); // Αναμονή για τη σύνδεση της σειριακής θύρας USB

  Serial.println("\n=============================================");
  Serial.println(" ESP32-S3 Motoron Dual Motor Diagnostic Test ");
  Serial.println("=============================================");

  // 1. Ενεργοποίηση της γραμμής ισχύος 12V
  pinMode(POWER_ENABLE, OUTPUT);
  digitalWrite(POWER_ENABLE, HIGH);
  delay(300);

  // 2. Αρχικοποίηση του διαύλου I2C
  Wire.begin(SDA_PIN, SCL_PIN); 
  mc.reinitialize();
  mc.clearResetFlag();

  // 3. Απενεργοποίηση του command timeout
  mc.disableCommandTimeout();

  // 4. Ρύθμιση ρυθμών επιτάχυνσης/επιβράδυνσης και για τους δύο κινητήρες
  mc.setErrorResponse(MOTORON_ERROR_RESPONSE_COAST);
  
  // Δεξιός κινητήρας (Κανάλι 1)
  mc.setMaxAcceleration(RIGHT_MOTOR_CH, 100);
  mc.setMaxDeceleration(RIGHT_MOTOR_CH, 100);
  
  // Αριστερός κινητήρας (Κανάλι 2)
  mc.setMaxAcceleration(LEFT_MOTOR_CH, 100);
  mc.setMaxDeceleration(LEFT_MOTOR_CH, 100);

  Serial.println("Watchdog disabled. Starting dual motor test loop...\n");
}

void loop() {
  mc.clearResetFlag();

  // 1. Κίνηση εμπρός (50% ισχύς = 400 / 800)
  Serial.println("[TEST] Both Motors: FORWARD");
  mc.setSpeed(RIGHT_MOTOR_CH, 400);
  mc.setSpeed(LEFT_MOTOR_CH, 400);
  checkMotoronErrors();
  delay(3000);

  // 2. Στάση
  Serial.println("[TEST] Both Motors: STOP");
  mc.setSpeed(RIGHT_MOTOR_CH, 0);
  mc.setSpeed(LEFT_MOTOR_CH, 0);
  delay(1000);

  // 3. Κίνηση πίσω (50% ισχύς = -400 / 800)
  Serial.println("[TEST] Both Motors: REVERSE");
  mc.setSpeed(RIGHT_MOTOR_CH, -400);
  mc.setSpeed(LEFT_MOTOR_CH, -400);
  checkMotoronErrors();
  delay(3000);

  // 4. Στάση
  Serial.println("[TEST] Both Motors: STOP");
  mc.setSpeed(RIGHT_MOTOR_CH, 0);
  mc.setSpeed(LEFT_MOTOR_CH, 0);
  delay(1000);
}