// Main entry point for the master Arduino codebase.

#include "src/Config/Pins.h"
#include "src/Motors/MotorControl.h"
#include "src/Sensors/Ultrasonic.h"
#include "src/Arm/ScanArm.h"
#include "src/Sensors/Mpu6050.h"
#include "src/Comm/SlaveLink.h"
#include "src/Logic/ScanWanderLogic.h"

static void printCountdown() {
  Serial.println(F("Starting in 3..."));
  delay(1000);
  Serial.println(F("2..."));
  delay(1000);
  Serial.println(F("1..."));
  delay(1000);
}

void setup() {
  Serial.begin(9600);
  Serial.println(F("=== Master: Scan Wander ==="));

  initMotors();
  initUltrasonic(PIN_TRIG_FRONT, PIN_ECHO_FRONT);
  initUltrasonic(PIN_TRIG_SHOULDER, PIN_ECHO_SHOULDER);
  initScanArm();
  initMpu();
  initSlaveLink();

  randomSeed(analogRead(PIN_RANDOM_SEED));
  printCountdown();
  enableMotors();
}

void loop() {
  runScanWanderLoop();
}
