// Actual motor driver writes happen here.

#include "MotorControl.h"
#include "../Config/Pins.h"
#include "../Config/DriveConfig.h"

void initMotors() {
  pinMode(PIN_STBY, OUTPUT);
  pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_PWMB, OUTPUT);
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);

  stopMotors();
  digitalWrite(PIN_STBY, LOW);
}

void enableMotors() {
  digitalWrite(PIN_STBY, HIGH);
}

void disableMotors() {
  stopMotors();
  digitalWrite(PIN_STBY, LOW);
}

void setRightTrack(bool forward, uint8_t speed) {
  bool pinHigh = (forward == RIGHT_FORWARD_HIGH);
  digitalWrite(PIN_AIN1, pinHigh ? HIGH : LOW);
  analogWrite(PIN_PWMA, speed);
}

void setLeftTrack(bool forward, uint8_t speed) {
  bool pinHigh = (forward == LEFT_FORWARD_HIGH);
  digitalWrite(PIN_BIN1, pinHigh ? HIGH : LOW);
  analogWrite(PIN_PWMB, speed);
}

void stopMotors() {
  analogWrite(PIN_PWMA, 0);
  analogWrite(PIN_PWMB, 0);
}
