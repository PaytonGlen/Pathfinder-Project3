// Shared HC-SR04 helper functions.

#include "Ultrasonic.h"

static const unsigned long ECHO_TIMEOUT_US = 30000;

void initUltrasonic(uint8_t trigPin, uint8_t echoPin) {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);
}

int readDistanceCm(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long pulseDuration = pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);
  if (pulseDuration == 0) return 0;
  return pulseDuration / 58;
}
