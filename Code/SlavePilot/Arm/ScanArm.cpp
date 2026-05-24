// Servo control code for the scan arm.

#include "ScanArm.h"
#include "../Config/Pins.h"
#include <Servo.h>

static Servo panServo;
static Servo tiltServo;

void initScanArm() {
  panServo.attach(PIN_SERVO_PAN);
  tiltServo.attach(PIN_SERVO_TILT);
  panServo.write(PAN_CENTER);
  tiltServo.write(TILT_LEVEL);
}

void panTo(int angle) {
  panServo.write(angle);
}

void tiltTo(int angle) {
  tiltServo.write(angle);
}

void centerPan() {
  panServo.write(PAN_CENTER);
  delay(SERVO_SETTLE_MS);
}
