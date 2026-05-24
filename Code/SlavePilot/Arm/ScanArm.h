// Shared scan arm helpers.

#ifndef SCAN_ARM_H
#define SCAN_ARM_H

#include <Arduino.h>

// These are angles (Degrees) calibrated to turn the arm left/right and up/down
const int PAN_CENTER = 50;      // This centers the arm to face forward
const int PAN_LEFT = 110;
const int PAN_RIGHT = 0;
const int TILT_LEVEL = 160;     // This controls vertical motion

const unsigned long SERVO_SETTLE_MS = 400;

void initScanArm();
void panTo(int angle);
void tiltTo(int angle);
void centerPan();

#endif
