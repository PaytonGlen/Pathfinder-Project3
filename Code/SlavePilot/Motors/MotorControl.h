// Low-level motor pin helpers.

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

void initMotors();
void enableMotors();
void disableMotors();

void setRightTrack(bool forward, uint8_t speed);
void setLeftTrack(bool forward, uint8_t speed);
void stopMotors();

#endif
