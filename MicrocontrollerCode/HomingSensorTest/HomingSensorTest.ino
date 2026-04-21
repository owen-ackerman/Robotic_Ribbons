// HomingSensorTest.ino
// Reads all 6 homing limit switch pins and prints their states to Serial.
// No motors, no homing routine — diagnostics only.

const int homingPins[6] = {33, 37, 41, 45, 49, 53};

void setup() {
  Serial.begin(115200);
  while (!Serial);

  for (int i = 0; i < 6; i++) {
    pinMode(homingPins[i], INPUT);  // no internal pullup — matches external wiring
  }

  Serial.println("=== Homing Sensor Test ===");
  Serial.println("Pin mode: INPUT (no pullup)");
  Serial.println("Printing sensor states every 250ms...");
  Serial.println();
}

void loop() {
  Serial.print("R1:");
  Serial.print(digitalRead(homingPins[0]));
  Serial.print("  R2:");
  Serial.print(digitalRead(homingPins[1]));
  Serial.print("  R3:");
  Serial.print(digitalRead(homingPins[2]));
  Serial.print("  R4:");
  Serial.print(digitalRead(homingPins[3]));
  Serial.print("  R5:");
  Serial.print(digitalRead(homingPins[4]));
  Serial.print("  R6:");
  Serial.println(digitalRead(homingPins[5]));

  delay(250);
}
