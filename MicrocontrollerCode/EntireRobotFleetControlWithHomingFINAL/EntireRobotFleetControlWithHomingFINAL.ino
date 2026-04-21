#include <AccelStepper.h>
#include <elapsedMillis.h>
#include <Servo.h>
//#include <algorithm>

// ── Pin Definitions ──────────────────────────────────────────────────────────
const int servoPins[6] = {2,  4,  6,  8,  10, 12};
const int stepPins[6]  = {3,  5,  7,  9,  11, 13};
const int dirPins[6]   = {26, 27, 28, 29, 32, 31};
const int stepperDir[6] = {1, 1, -1, 1, 1, 1};  // flip index 2

// Limit switch pins for each of the 6 steppers
const int homingPins[6] = {33, 37, 41, 45, 49, 53};

// ── Homing Config ────────────────────────────────────────────────────────────
#define HOMING_SPEED 2000   // steps/sec
#define HOMING_DIR   -1     // -1 or 1 depending on which direction homes

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

// ── Homing ───────────────────────────────────────────────────────────────────
void homeAllSteppers() {
  Serial.println("Homing sequence started...");

  // Detach servos during homing to prevent Timer 1 conflict with step pins 11/12/13
  for (int i = 0; i < 6; i++) {
    servos[i].detach();
  }

  bool homed[6] = {false, false, false, false, false, false};

  for (int i = 0; i < 6; i++) {
    steppers[i].setSpeed(HOMING_SPEED * HOMING_DIR * stepperDir[i]);
  }

  // Run motors for 100ms before checking sensors so any stepper
  // already sitting on a switch moves clear of it first
  unsigned long backoffStart = millis();
  while (millis() - backoffStart < 300) {
    for (int i = 0; i < 6; i++) {
      steppers[i].runSpeed();
    }
  }

  Serial.println("Reading sensors now");

  bool allHomed = false;
  while (!allHomed) {
    allHomed = true;

    for (int i = 0; i < 6; i++) {
      if (homed[i]) continue;

      steppers[i].runSpeed();

      // Sensors are active-low: LOW = triggered
      if (digitalRead(homingPins[i]) == LOW) {
        steppers[i].setSpeed(0);
        steppers[i].setCurrentPosition(0);
        homed[i] = true;
      }

      if (!homed[i]) allHomed = false;
    }
  }

  Serial.println("All steppers homed.");

  // Re-attach servos and restore neutral position after homing is complete
  for (int i = 0; i < 6; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(0);
  }
}

// ── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial);

  for (int i = 0; i < 6; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(90);
    steppers[i].setMaxSpeed(100000.0);
    steppers[i].setSpeed(0);
    pinMode(homingPins[i], INPUT);
  }

  Serial.println("Hello Serial Setup");
  homeAllSteppers();
}

// ── Loop ─────────────────────────────────────────────────────────────────────
//
// Fixed 8-byte packet format (TouchDesigner must match):
//   [0xAA][0x55][robotNum 1-6][motorType][b0][b1][b2][b3]
//
//   Stepper (motorType=2): b0=high, b1=med, b2=low, b3=sign(0/1)
//     → SPS = (b0<<16)|(b1<<8)|b2, negated if b3==1
//   Servo   (motorType=1): b0=angleHigh, b1=angleLow, b2=0, b3=0
//     → angle = ((b0<<8)|b1) - 1
//
void loop() {
  // Run all steppers on every loop — this must not be blocked
  for (int i = 0; i < 6; i++) {
    steppers[i].runSpeed();
  }

  // Scan for the two-byte sync header 0xAA 0x55.
  // We only scan while >= 8 bytes are available so we never consume the
  // header without its payload — doing so would orphan the payload bytes
  // and stall communication permanently.
  bool headerFound = false;
  while (Serial.available() >= 8) {
    if (Serial.peek() == 0xAA) {
      Serial.read();               // tentatively consume 0xAA
      if (Serial.peek() == 0x55) {
        Serial.read();             // consume 0x55 — full header confirmed
        headerFound = true;
        break;
      }
      // 0xAA was a data byte, not a real header — keep scanning
    } else {
      Serial.read();               // discard non-sync byte
    }
  }

  if (!headerFound) return;

  // Header consumed; 6 payload bytes are guaranteed available (we had >= 8)
  int robotNum = Serial.read() - 1;  // convert 1-based to 0-based
  int motorNum = Serial.read();
  int b0       = Serial.read();
  int b1       = Serial.read();
  int b2       = Serial.read();
  int b3       = Serial.read();

  // Validate after reading all bytes so nothing is left stranded in the buffer
  if (robotNum < 0 || robotNum > 5) return;
  if (motorNum != 1 && motorNum != 2) return;

  if (motorNum == 2) {
    int stepperSPS = (b0 << 16) | (b1 << 8) | b2;
    if (b3 == 1) stepperSPS = -stepperSPS;
    steppers[robotNum].setSpeed(stepperSPS * stepperDir[robotNum]);

  } else if (motorNum == 1) {
    int servoAngle = ((b0 << 8) | b1) - 1;
    servos[robotNum].write(servoAngle);
  }
}
