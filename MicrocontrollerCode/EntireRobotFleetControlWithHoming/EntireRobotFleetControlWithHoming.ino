#include <AccelStepper.h>
#include <elapsedMillis.h>
#include <Servo.h>
//#include <algorithm>

// ── Pin Definitions ──────────────────────────────────────────────────────────
// Stepper pins: {stepPin, dirPin} for each of the 6 steppers
const int servoPins[6] = {2,  4,  6,  8,  10, 12};
// Servo pins for each of the 6 servos
const int stepPins[6] = {3, 5, 7, 9, 11, 13};
const int dirPins[6]  = {26, 27, 28, 29, 32, 31};

// Limit switch pins for each of the 6 steppers
const int homingPins[6] = {33, 37, 41, 45, 49, 53};  // swap for your actual pins

bool printBool = true;
// ── Homing Config ────────────────────────────────────────────────────────────
#define HOMING_SPEED  500   // steps/sec — slow enough to not slam the switch
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
elapsedMillis printTime;
bool allTrue(bool arr[], int len) {
    for (int i = 0; i < len; i++) {
        if (!arr[i]) return false;
    }
    return true;
}

// ── Homing ───────────────────────────────────────────────────────────────────
void homeAllSteppers() {
  Serial.println("Homing sequence started...");

  bool homed[6]     = {false, false, false, false, false, false};
  bool prevState[6] = {true,  true,  true,  true,  true,  true };  // HIGH = unpressed
  bool allHomed     = false;

  // Start all steppers moving toward home
  for (int i = 0; i < 6; i++) {
    Serial.print("Setting stepper speed for motor: ");
    Serial.println(i);
    steppers[i].setSpeed(HOMING_SPEED * HOMING_DIR);
  }

  while (!allHomed) {
    allHomed = allTrue(homed, 6);
   

    for (int i = 0; i < 6; i++) {
      if (homed[i]) continue;

      // Keep stepping motors that aren't homed yet
      steppers[i].runSpeed();

      // Detect falling edge: was HIGH, now LOW
      bool curState = digitalRead(homingPins[i]);
      if (prevState[i] == HIGH && curState == LOW) {
        steppers[i].setSpeed(0);
        steppers[i].setCurrentPosition(0);  // zero position at the switch
        homed[i] = true;

        Serial.print("Robot ");
        Serial.print(i + 1);
        Serial.println(" | Stepper | Homed");
      }

      prevState[i] = curState;
      if (!homed[i]) allHomed = false;
    }
  }

  Serial.println("All steppers homed.");
}

// ── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(250000);
  while (!Serial);

  for (int i = 0; i < 6; i++) {
    servos[i].attach(servoPins[i]);
    steppers[i].setMaxSpeed(100000.0);
    steppers[i].setSpeed(0);
    pinMode(homingPins[i], INPUT_PULLUP);
  }
  
  Serial.println("Hello Serial Setup");


  //homeAllSteppers();
}

// ── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
 // steppers[3].setSpeed(500);
  // Run all steppers on every loop — this must not be blocked
  for (int i = 0; i < 6; i++) {
    steppers[i].runSpeed();
  }

  // Serial packet parsing
  // Stepper packet: RobotNum, MotorNum(2), high, med, low, sign  → 6 bytes
  // Servo packet:   RobotNum, MotorNum(1), high, low             → 4 bytes
  while (Serial.available() >= 4) {
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
      if (printBool) {
        Serial.print("Robot ");
        Serial.print(robotNum + 1);
        Serial.print(" | Stepper | SPS: ");
        Serial.println(stepperSPS);
      }

    } else if (motorNum == 1) {
      // Servo — needs 2 more bytes
      if (Serial.available() < 2) return;
      int high = Serial.read();
      int low  = Serial.read();

      int servoAngle = ((high << 8) | low) - 1;

      servos[robotNum].write(servoAngle);
      
      if (printBool) {
      Serial.print("Robot ");
      Serial.print(robotNum + 1);
      Serial.print(" | Servo | Angle: ");
      Serial.println(servoAngle);
      }
    }
  }
}
