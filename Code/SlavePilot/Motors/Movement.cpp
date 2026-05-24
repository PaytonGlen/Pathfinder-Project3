// Shared movement helper implementations.

#include "Movement.h"
#include "MotorControl.h"
#include "../Config/DriveConfig.h"

void driveForward(unsigned long durationMs) {
  setRightTrack(true, RIGHT_DRIVE_SPEED);
  setLeftTrack(true, LEFT_DRIVE_SPEED);
  delay(durationMs);
  stopMotors();
}

void reverse(unsigned long durationMs) {
  setRightTrack(false, RIGHT_DRIVE_SPEED);
  setLeftTrack(false, LEFT_DRIVE_SPEED);
  delay(durationMs);
  stopMotors();
}

void turnRight(unsigned long durationMs) {
  setLeftTrack(true, TURN_SPEED);
  setRightTrack(false, TURN_SPEED);
  delay(durationMs);
  stopMotors();
}

void turnLeft(unsigned long durationMs) {
  setLeftTrack(false, TURN_SPEED);
  setRightTrack(true, TURN_SPEED);
  delay(durationMs);
  stopMotors();
}
