#include <AccelStepper.h>
#include <elapsedMillis.h>
#include <Servo.h>

// ── Pin Definitions ──────────────────────────────────────────────────────────
// Stepper pins: {stepPin, dirPin} for each of the 6 steppers
const int stepPins[6] = {2,  4,  6,  8,  10, 12};
const int dirPins[6]  = {22, 23, 24, 25, 26, 27};

// Servo pins for each of the 6 servos
const int servoPins[6] = {3, 5, 7, 9, 11, 13};

// ── Motor Objects ────────────────────────────────────────────────────────────
AccelStepper steppers[6] = {
  AccelStepper(AccelStepper::FULL2WIRE, stepPins[0], dirPins[0]),
  AccelStepper(AccelStepper::FULL2WIRE, stepPins[1], dirPins[1]),
  AccelStepper(AccelStepper::FULL2WIRE, stepPins[2], dirPins[2]),
  AccelStepper(AccelStepper::FULL2WIRE, stepPins[3], dirPins[3]),
  AccelStepper(AccelStepper::FULL2WIRE, stepPins[4], dirPins[4]),
  AccelStepper(AccelStepper::FULL2WIRE, stepPins[5], dirPins[5]),
};

Servo servos[6];

elapsedMillis printTime;

// ── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 6; i++) {
    servos[i].attach(servoPins[i]);

    steppers[i].setMaxSpeed(100000.0);
    steppers[i].setSpeed(0);
  }
}

// ── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
  // Run all steppers on every loop — this must not be blocked
  for (int i = 0; i < 6; i++) {
    steppers[i].runSpeed();
  }

  // Serial packet parsing
  // Stepper packet: RobotNum, MotorNum(2), high, med, low, sign  → 6 bytes
  // Servo packet:   RobotNum, MotorNum(1), high, low             → 4 bytes
  if (Serial.available() >= 4) {
    int robotNum = Serial.read();  // 1–6
    int motorNum = Serial.read();  // 1 = servo, 2 = stepper

    // Validate robot index before using it
    robotNum = robotNum - 1;  // convert 1-based to 0-based
    if (robotNum < 0 || robotNum > 5) return;

    if (motorNum == 2) {
      // Stepper — needs 4 more bytes
      if (Serial.available() < 4) return;
      int high = Serial.read();
      int med  = Serial.read();
      int low  = Serial.read();
      int sign = Serial.read();

      int stepperSPS = (high << 16) | (med << 8) | low;
      if (sign == 1) stepperSPS = -stepperSPS;

      steppers[robotNum].setSpeed(stepperSPS);
      
      Serial.print("Robot ");
      Serial.print(robotNum);
      Serial.print(" | Stepper | SPS: ");
      Serial.println(stepperSPS);

    } else if (motorNum == 1) {
      // Servo — needs 2 more bytes
      if (Serial.available() < 2) return;
      int high = Serial.read();
      int low  = Serial.read();

      int servoAngle = ((high << 8) | low) - 1;
      
      servos[robotNum].write(servoAngle);

      Serial.print("Robot ");
      Serial.print(robotNum);
      Serial.print(" | Servo | Angle: ");
      Serial.println(servoAngle);
    }
  }
}